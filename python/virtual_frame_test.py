#!/usr/bin/env python

from cartesian_interface.pyci_all import *
import rospy
import math
import numpy as np


def main():

    # obtain ci ros client
    ci = pyci.CartesianInterfaceRos()

    # define time between waypoints
    time = 3.0

    # angles for later use
    angle_1 = 15.0
    angle_2 = 30.0
    sin_1 = math.sin(angle_1/2.0/180.0*math.pi)
    cos_1 = math.cos(angle_1/2.0/180.0*math.pi)
    sin_2 = math.sin(angle_2/2.0/180.0*math.pi)
    cos_2 = math.cos(angle_2/2.0/180.0*math.pi)

    # initial virtual frame pose
    vf_pose_initial, _, _ = ci.getPoseReference('car_frame')

    # spin base
    ci.setControlMode('car_frame', pyci.ControlType.Velocity)
    vref = np.array([0.05, 0, 0, 0, 0, 0])
    ci.setVelocityReferenceAsync('car_frame', vref, -1.0)

    # pelvis down
    print 'Pelvis down..'
    ci.setTargetPose('pelvis', Affine3(pos=[0, 0, -0.1]), time)
    ci.waitReachCompleted('pelvis')
    base_pose_initial, _, _ = ci.getPoseReference('pelvis')

    # narrow support
    print 'Narrow support..'
    ci.update()
    b_t_w = ci.getPoseReference('wheel_1')[0]
    ci.setTargetPose('wheel_1', Affine3(pos=[0.5,  0.25,  b_t_w.translation[2]]), time)

    b_t_w = ci.getPoseReference('wheel_2')[0]
    ci.setTargetPose('wheel_2', Affine3(pos=[0.5, -0.25,  b_t_w.translation[2]]), time)

    b_t_w = ci.getPoseReference('wheel_3')[0]
    ci.setTargetPose('wheel_3', Affine3(pos=[-0.5, 0.25,  b_t_w.translation[2]]), time)

    b_t_w = ci.getPoseReference('wheel_4')[0]
    ci.setTargetPose('wheel_4', Affine3(pos=[-0.5, -0.25, b_t_w.translation[2]]), time)

    ci.waitReachCompleted('wheel_4')
    rospy.sleep(1.0)

    # wide support
    print 'Wide support..'
    b_t_w = ci.getPoseReference('wheel_1')[0]
    ci.setTargetPose('wheel_1', Affine3(pos=[0.3, 0.5, b_t_w.translation[2]]), time)

    b_t_w = ci.getPoseReference('wheel_2')[0]
    ci.setTargetPose('wheel_2', Affine3(pos=[0.3, -0.5, b_t_w.translation[2]]), time)

    b_t_w = ci.getPoseReference('wheel_3')[0]
    ci.setTargetPose('wheel_3', Affine3(pos=[-0.3, 0.5, b_t_w.translation[2]]), time)

    b_t_w = ci.getPoseReference('wheel_4')[0]
    ci.setTargetPose('wheel_4', Affine3(pos=[-0.3, -0.5, b_t_w.translation[2]]), time)

    ci.waitReachCompleted('wheel_4')
    rospy.sleep(1.0)

    # homing support
    print 'Homing support..'
    b_t_w = ci.getPoseReference('wheel_1')[0]
    ci.setTargetPose('wheel_1', Affine3(pos=[0.35, 0.35, b_t_w.translation[2]]), time)

    b_t_w = ci.getPoseReference('wheel_2')[0]
    ci.setTargetPose('wheel_2', Affine3(pos=[0.35, -0.35, b_t_w.translation[2]]), time)

    b_t_w = ci.getPoseReference('wheel_3')[0]
    ci.setTargetPose('wheel_3', Affine3(pos=[-0.35, 0.35, b_t_w.translation[2]]), time)

    b_t_w = ci.getPoseReference('wheel_4')[0]
    ci.setTargetPose('wheel_4', Affine3(pos=[-0.35, -0.35, b_t_w.translation[2]]), time)

    ci.waitReachCompleted('wheel_4')
    rospy.sleep(1.0)

    # left-right with waist
    print 'Pelvis left right..'
    ci.update()
    b_t_p = ci.getPoseReference('pelvis')[0]
    print 'Waist pose is ', b_t_p

    ci.setTargetPose('pelvis', Affine3(pos=[0.0, 0.10, b_t_p.translation[2]]), time)
    ci.waitReachCompleted('pelvis')

    ci.setTargetPose('pelvis', Affine3(pos=[0.0, -0.10, b_t_p.translation[2]]), time)
    ci.waitReachCompleted('pelvis')

    # front-back with waist
    print 'Pelvis front back..'
    ci.setTargetPose('pelvis', Affine3(pos=[0.15, 0.0, b_t_p.translation[2]]), time)
    ci.waitReachCompleted('pelvis')

    ci.setTargetPose('pelvis', Affine3(pos=[-0.15, 0.0, b_t_p.translation[2]]), time)
    ci.waitReachCompleted('pelvis')

    ci.setTargetPose('pelvis', Affine3(pos=[0.0, 0.0, b_t_p.translation[2]]), time)
    ci.waitReachCompleted('pelvis')

    # turn left-right with waist
    print 'Pelvis rotate..'

    ci.setBaseLink('arm1_8', 'car_frame')
    ci.setBaseLink('arm2_8', 'car_frame')

    ci.setTargetPose('pelvis', Affine3(rot=[0, 0,  sin_1, cos_1]), time/2.0, True)
    ci.waitReachCompleted('pelvis')

    ci.setTargetPose('pelvis', Affine3(rot=[0, 0, -sin_2, cos_2]), time/2.0, True)
    ci.waitReachCompleted('pelvis')

    ci.setTargetPose('pelvis', Affine3(rot=[0, 0,  sin_1, cos_1]), time/2.0, True)
    ci.waitReachCompleted('pelvis')

    # ramp down virtual frame speed
    t = 0
    dt = 0.01
    tstop = time

    print 'Stopping rolling..'
    while t < tstop:
        vref_t = vref * (tstop - t)/tstop
        ci.setVelocityReference('car_frame', vref_t)
        rospy.sleep(dt)
        t += dt

    # ci.stopVelocityReferenceAsync('car_frame')

    ci.setBaseLink('arm1_8', 'torso_2')
    ci.setBaseLink('arm2_8', 'torso_2')
    ci.setControlMode('car_frame', pyci.ControlType.Position)

    raw_input('Press ENTER to continue')

    # base w.r.t. world and move the virtual frame
    print 'Move virtual frame with fixed pelvis..'
    ci.setBaseLink('pelvis', 'world')

    ci.setTargetPose('car_frame', Affine3(pos=[0.15, 0, 0]), time, True)
    ci.waitReachCompleted('car_frame')
    ci.setTargetPose('car_frame', Affine3(pos=[-0.30, 0, 0]), time, True)
    ci.waitReachCompleted('car_frame')
    ci.setTargetPose('car_frame', Affine3(pos=[0.15, 0, 0]), time, True)
    ci.waitReachCompleted('car_frame')

    ci.setTargetPose('car_frame', Affine3(pos=[0, 0.075, 0]), time, True)
    ci.waitReachCompleted('car_frame')
    ci.setTargetPose('car_frame', Affine3(pos=[0, -0.15, 0]), time, True)
    ci.waitReachCompleted('car_frame')
    ci.setTargetPose('car_frame', Affine3(pos=[0, 0.075, 0]), time, True)
    ci.waitReachCompleted('car_frame')

    ci.setTargetPose('car_frame', Affine3(rot=[0, 0,  sin_1, cos_1]), time, True)
    ci.waitReachCompleted('car_frame')
    ci.setTargetPose('car_frame', Affine3(rot=[0, 0, -sin_2, cos_2]), time, True)
    ci.waitReachCompleted('car_frame')
    ci.setTargetPose('car_frame', Affine3(rot=[0, 0,  sin_1, cos_1]), time, True)
    ci.waitReachCompleted('car_frame')

    # base w.r.t virtual frame
    ci.setBaseLink('pelvis', 'car_frame')

    # # base and virtual frame to starting pose
    # ci.setTargetPose('car_frame', vf_pose_initial, time)
    # ci.setTargetPose('pelvis', base_pose_initial, time)

    raw_input('Press ENTER to continue')

    # ee w.r.t. world
    print 'Chicken head..'
    ci.setBaseLink('arm1_8', 'world')
    ci.setBaseLink('arm2_8', 'world')
    ci.setControlMode('pelvis', pyci.ControlType.Disabled)

    print 'High manipulation..'
    ci.setTargetPose('arm1_8', Affine3(pos=[0.0, 0, 0.4]), time, True)
    ci.setTargetPose('arm2_8', Affine3(pos=[0.0, 0, 0.4]), time, True)
    ci.waitReachCompleted('arm1_8')
    ci.waitReachCompleted('arm2_8')

    wp_list = list()
    wp_list.append(pyci.WayPoint(Affine3(pos=[-0.30, 0, 0]), time))
    wp_list.append(pyci.WayPoint(Affine3(pos=[-0.30, 0, 0], rot=[0, 0,  sin_2, cos_2]), 2*time))
    wp_list.append(pyci.WayPoint(Affine3(pos=[-0.30, 0, 0], rot=[0, 0, -sin_2, cos_2]), 3*time))
    wp_list.append(pyci.WayPoint(Affine3(pos=[-0.30, 0, 0], rot=[0, 0, 0, 1]), 4*time))
    wp_list.append(pyci.WayPoint(Affine3(pos=[0.0, 0, 0]), 5*time))

    ci.setWaypoints('car_frame', wp_list, True)
    ci.waitReachCompleted('car_frame')


    ci.setTargetPose('arm1_8', Affine3(pos=[-0.0, 0, -0.4]), time, True)
    ci.setTargetPose('arm2_8', Affine3(pos=[-0.0, 0, -0.4]), time, True)
    ci.waitReachCompleted('arm1_8')
    ci.waitReachCompleted('arm2_8')

    ci.setControlMode('pelvis', pyci.ControlType.Position)

    ci.setTargetPose('pelvis', Affine3(pos=[0.1, 0, 0]), time, True)
    ci.waitReachCompleted('pelvis')

    ci.setTargetPose('wheel_4', Affine3(pos=[0, 0, 0.35]), time/2., True)
    ci.waitReachCompleted('wheel_4')

    ci.setTargetPose('wheel_4', Affine3(pos=[0, 0, -0.35]), time/2., True)
    ci.waitReachCompleted('wheel_4')

    ci.setTargetPose('pelvis', Affine3(pos=[-0.1, 0, 0]), time, True)
    ci.waitReachCompleted('pelvis')



if __name__ == '__main__':
    rospy.init_node('virtual_frame_test')
    main()
