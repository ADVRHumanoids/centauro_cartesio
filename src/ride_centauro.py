#!/usr/bin/python2.7

import numpy as np
import time
import rospy
from std_msgs.msg import Float64
import rospkg
import scipy.io 
import yaml
from os.path import join, dirname, abspath
import os 
import argparse
from math import cos, sin, trunc

os.environ['XBOT_VERBOSE'] = '2'
from xbot_interface import xbot_interface as xbi 
from xbot_interface import config_options as co 
from cartesian_interface.pyci_all import *

# some utility functions
def get_ci(model, ikpb, dt):
    return pyci.CartesianInterface.MakeInstance('OpenSot', ikpb, model, dt)

def get_ikpb_car_model():
    rospack = rospkg.RosPack()
    ikpath = join(rospack.get_path('centauro_cartesio'), 'configs/centauro_car_model_stack.yaml')
    with open(ikpath, 'r') as f:
        return f.read()

def get_car_model_urdf_srdf():
    rospack = rospkg.RosPack()
    centauro_urdf_path = join(rospack.get_path('centauro_urdf'), 'urdf/centauro_virtual_frame.urdf')
    centauro_srdf_path = join(rospack.get_path('centauro_srdf'), 'srdf/centauro_virtual_frame.srdf')
    paths = [centauro_urdf_path, centauro_srdf_path] 
    return (open(p, 'r').read() for p in paths) 
    
def update_ik(ci, model, time, dt):
    ci.update(time, dt)
    q = model.getJointPosition()
    qdot = model.getJointVelocity()
    q += qdot * dt 
    model.setJointPosition(q)
    model.update()
    return q, qdot

def get_xbot_cfg(urdf, srdf):
    cfg = co.ConfigOptions()
    cfg.set_urdf(urdf)
    cfg.set_srdf(srdf)
    cfg.generate_jidmap()
    cfg.set_string_parameter('model_type', 'RBDL')
    cfg.set_string_parameter('framework', 'ROS')
    cfg.set_bool_parameter('is_model_floating_base', True)
    return cfg

def quintic(alpha):
    if alpha < 0:
        return 0
    elif alpha > 1:
        return 1
    else:
        return ((6*alpha - 15)*alpha + 10)*alpha**3


# initialize ros
rospy.init_node('ride_centauro')

# get augmented kinematic model for ik
model_cfg = get_xbot_cfg(*get_car_model_urdf_srdf())
model = xbi.ModelInterface(model_cfg)

# get robot from standard urdf
ctrl_cfg = get_xbot_cfg(rospy.get_param('/xbotcore/robot_description'), 
                        rospy.get_param('/xbotcore/robot_description_semantic')) 
robot = xbi.RobotInterface(ctrl_cfg)
robot.sense()

# set control mode

# whole robot
robot.setControlMode(xbi.ControlMode.PosImpedance())

# wheels and velodyne
ctrlmode = {'j_wheel_{}'.format(i+1) : xbi.ControlMode.Velocity() for i in range(4)}
ctrlmode['neck_velodyne'] = xbi.ControlMode.Velocity()
robot.setControlMode(ctrlmode)


# set model state a
qref = robot.eigenToMap(robot.getPositionReference())
model.setJointPosition(qref)
model.update()


# visualize solution
rspub = pyci.RobotStatePublisher(model)

# get ci
dt = 0.01
ci = get_ci(model, get_ikpb_car_model(), dt)

# define tasks
time_t = 0.0
done = False

# set lower impedance for "right arm interface"
robot.setStiffness({"j_arm2_3": 100})
robot.setStiffness({"j_arm2_4": 100})
robot.move()

# safe sleep
time.sleep(2)

#starting pose
robot.sense()
zero_pose = robot.model().getPose('arm2_8')
print zero_pose

# prepare ROS stuffs
pub_cart_error_x = rospy.Publisher('cart_error_x', Float64, queue_size=10)
pub_cart_error_y = rospy.Publisher('cart_error_y', Float64, queue_size=10)

car_vel_ref_x = rospy.Publisher('car_vel_ref_x', Float64, queue_size=10)
car_vel_ref_teta = rospy.Publisher('car_vel_ref_teta', Float64, queue_size=10)

car = ci.getTask('car_frame')

print('started looping..')
rate = rospy.Rate(1./dt * 1.0)

while not rospy.is_shutdown():

    car_vel_ref = np.zeros((6))

    T_car_frame = model.getPose("car_frame")

    robot.sense()
    cart_error_x = zero_pose.translation[0] - robot.model().getPose('arm2_8').translation[0]
    cart_error_y = zero_pose.translation[1] - robot.model().getPose('arm2_8').translation[1]

    pub_cart_error_x.publish(cart_error_x)
    pub_cart_error_y.publish(cart_error_y)

    k = 3
    max_vel_x = 0.25
    max_vel_teta = 0.2

    car_vel_ref[0:3] = -(k * cart_error_x) * T_car_frame.linear[:,0]
    car_vel_ref[5] = -(k * cart_error_y)

    # saturation
    if abs(car_vel_ref[0]) > max_vel_x:
        car_vel_ref[0] = np.sign(car_vel_ref[0]) * max_vel_x

    if abs(car_vel_ref[5]) > max_vel_teta:
        car_vel_ref[5] = np.sign(car_vel_ref[5]) * max_vel_teta

    car_vel_ref_x.publish(car_vel_ref[0])
    car_vel_ref_teta.publish(car_vel_ref[5])

    car.setControlMode(pyci.ControlType.Velocity)
    car.setVelocityReference(car_vel_ref)

    # run ik
    update_ik(ci, model, time_t, dt)
    rspub.publishTransforms('ride_demo')

    # send model state as reference
    qref = model.getJointPositionMap()
    dqref = model.eigenToMap(model.getJointVelocity())

    robot.setPositionReference(qref)
    robot.setVelocityReference(dqref)
    robot.move()

    # sleep
    rate.sleep()
    time_t += dt
    