#!/usr/bin/python2.7

import numpy as np
import rospy
import rospkg
import scipy.io 
import yaml
from os.path import join, dirname, abspath
import os 
import argparse
from math import cos, sin

os.environ['XBOT_VERBOSE'] = '2'
from xbot_interface import xbot_interface as xbi 
from xbot_interface import config_options as co 
from cartesian_interface.pyci_all import *

parser = argparse.ArgumentParser(description='Scripted demo showcasing Centauro\'s ik')
parser.add_argument('--visual', '-v', required=False, action='store_true', help='Run in visual-only mode')
parser.add_argument('--rate', '-r', required=False, type=float, help='If visual mode is set, play back motion at X times normal speed')
args = parser.parse_args()

if args.rate is not None and not args.visual:
    print('--rate option requires --visual')
    exit(1)

if args.rate is None:
    args.rate = 1.0

# some utility functions
def get_ci(model, ikpb, dt):
    return pyci.CartesianInterface.MakeInstance('OpenSot', ikpb, model, dt)

def get_ikpb_car_model():
    ikpath = join(dirname(abspath(__file__)), 'poses2_ik.yaml')
    with open(ikpath, 'r') as f:
        return f.read()

def get_ikpb_basic():
    ikpath = join(dirname(abspath(__file__)), 'poses2_collision_avoidance_stack.yaml')
    with open(ikpath, 'r') as f:
        return f.read()

def get_car_model_urdf_srdf():
    rospack = rospkg.RosPack()
    centauro_path = join(rospack.get_path('centauro_cartesio'), 'configs')
    paths = [join(centauro_path, xml, 'centauro_car.' + xml) for xml in ['urdf', 'srdf']] 
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

def all_wheels(fl):
    signs = map(np.array, ([1, 1], [1, -1], [-1, 1], [-1, -1]))
    fl_mul = lambda sign: fl*sign 
    return map(fl_mul, signs)

def quintic(alpha):
    if alpha < 0:
        return 0
    elif alpha > 1:
        return 1
    else:
        return ((6*alpha - 15)*alpha + 10)*alpha**3


# initialize ros
rospy.init_node('poses2')

# get augmented kinematic model for ik
model_cfg = get_xbot_cfg(*get_car_model_urdf_srdf())
model = xbi.ModelInterface(model_cfg)

# get robot from standard urdf
ctrl_cfg = get_xbot_cfg(rospy.get_param('/xbotcore/robot_description'), 
                        rospy.get_param('/xbotcore/robot_description_semantic')) 
robot = xbi.RobotInterface(ctrl_cfg)
robot.sense()

# set control mode
robot.setControlMode(xbi.ControlMode.Position())
ctrlmode = {'j_wheel_{}'.format(i+1) : xbi.ControlMode.Velocity() for i in range(4)}
ctrlmode['neck_velodyne'] = xbi.ControlMode.Velocity()
robot.setControlMode(ctrlmode)

# set model state and fix base position
qref = robot.eigenToMap(robot.getPositionReference())
model.setJointPosition(qref)
model.update()

contact_names = ('wheel_' + str(i+1) for i in range(4))
floor_z = sum((model.getPose(c).translation[2] for c in contact_names)) / 4.0
Tfb = model.getFloatingBasePose()
Tfb.translation[2] -= floor_z
model.setFloatingBasePose(Tfb)
model.update()

# visualize solution
rspub = pyci.RobotStatePublisher(model)

# get ci
dt = 0.01
ci_car_frame = get_ci(model, get_ikpb_car_model(), dt)
ci_basic = get_ci(model, get_ikpb_basic(), dt)
ci = ci_car_frame

# get some useful tasks
vf = ci.getTask('car_frame')
base = ci.getTask('pelvis')
wheels = [ci.getTask('wheel_' + str(i+1)) for i in range(4)]
hands = [ci.getTask('arm{}_8'.format(i+1)) for i in range(2)]

# define tasks
time = 0.0
done = False

# shared data between states
class Data:
    pass

data = Data()

# define states
class noop:
    def __call__(self):
        pass

class pelvis_down:
    def __init__(self):
        base.setBaseLink('world')
        T, _, _ = base.getPoseReference()
        T.translation[0] -= 0.08
        T.translation[2] -= 0.10
        base.setPoseTarget(T, 2.0)
    def __call__(self):
        if base.getTaskState() == pyci.State.Online:
            return rotate_inplace()

class rotate_inplace:

    def __init__(self):
        
        self.angles = [0, 1.8, -1.8, 0]
        self.base_times = [0, 3.0, 7.0, 9.5]
        self.vf_times = [0.75, 3.25, 7.75, 9.5]
        self.t0 = time
        self.base_i = 0
        self.vf_i = 0

    def __call__(self):

        t = time - self.t0

        # play base trj
        i = self.base_i 
        j = self.base_i + 1

        # interpolate and set reference
        if j < len(self.angles):
            tau_base = (t - self.base_times[i])/(self.base_times[j] - self.base_times[i])
            alpha_base = quintic(tau_base)
            ang_base = self.angles[i] + (self.angles[j] - self.angles[i])*alpha_base
            Tbase = base.getPoseReference()[0]
            Tbase.quaternion = [0, 0, sin(ang_base/2), cos(ang_base/2)]
            base.setPoseReference(Tbase)

            if tau_base > 1.0:
                self.base_i += 1

        # play vf trj
        i = self.vf_i 
        j = self.vf_i + 1

        # interpolate and set reference
        if j < len(self.angles):
            tau_vf = (t - self.vf_times[i])/(self.vf_times[j] - self.vf_times[i])
            alpha_vf = quintic(tau_vf)
            ang_vf = self.angles[i] + (self.angles[j] - self.angles[i])*alpha_vf
            Tvf = vf.getPoseReference()[0]
            Tvf.quaternion = [0, 0, sin(ang_vf/2), cos(ang_vf/2)]
            vf.setPoseReference(Tvf)

            if tau_vf > 1.0:
                self.vf_i += 1

        if self.vf_i + 1 == len(self.angles) and self.base_i + 1 == len(self.angles):
            base.setBaseLink('car_frame')
            return compact_support()

class compact_support:

    def __call__(self):

        wh_pos = all_wheels(np.array([0.2, 0.2]))
        T, _, _ = base.getPoseReference()
        T.translation[0] += 0.08
        T.translation[2] += 0.10
        base.setPoseTarget(T, 2.0)

        for ref, w in zip(wh_pos, wheels):
            T, _, _ = w.getPoseReference()
            T.translation[:2] = ref 
            w.setPoseTarget(T, 3.0)

        return wait_tasks(wheels, lambda: wide_support())

class wide_support:

    def __call__(self):

        wh_pos = all_wheels(np.array([0.4, 0.6]))
        T, _, _ = base.getPoseReference()
        T.translation[2] -= 0.10
        base.setPoseTarget(T, 2.0)

        for ref, w in zip(wh_pos, wheels):
            T, _, _ = w.getPoseReference()
            T.translation[:2] = ref 
            w.setPoseTarget(T, 3.0)

        return wait_tasks(wheels, lambda: home_support())

class home_support:
    
    def __call__(self):

        wh_pos = all_wheels(np.array([0.35, 0.35]))

        for ref, w in zip(wh_pos, wheels):
            T, _, _ = w.getPoseReference()
            T.translation[:2] = ref 
            w.setPoseTarget(T, 6.0)

        return wait_tasks(wheels, lambda: high_manip())

class high_manip:

    Tstart = dict()

    def __call__(self):

        base.setActivationState(pyci.ActivationState.Disabled)
        for h in hands:
            h.setBaseLink('world')
            T, _, _ = h.getPoseReference()
            high_manip.Tstart[h.getName()] = T.copy()
            T.translation[2] += 0.5 
            h.setPoseTarget(T, 4.0)

        T, _, _ = vf.getPoseReference()
        high_manip.Tstart[vf.getName()] = T.copy()
        wpv = list()

        T.translation[0] -= 0.30
        wpv.append(pyci.WayPoint(T, 3.0))

        theta = 1.0
        R = Affine3(rot=[0, 0, sin(theta/2), cos(theta/2)])
        T.linear = np.matmul(R.linear, T.linear)
        wpv.append(pyci.WayPoint(T, 6.0))

        theta = -2.
        R = Affine3(rot=[0, 0, sin(theta/2), cos(theta/2)])
        T.linear = np.matmul(R.linear, T.linear)
        wpv.append(pyci.WayPoint(T, 9.0))

        theta = 1.
        R = Affine3(rot=[0, 0, sin(theta/2), cos(theta/2)])
        T.linear = np.matmul(R.linear, T.linear)
        wpv.append(pyci.WayPoint(T, 12.0))

        vf.setWayPoints(wpv)
        return wait_tasks([vf] + hands, lambda: restore_pose())

class restore_pose:
    def __call__(self):

        for h in hands:
            h.setPoseTarget(high_manip.Tstart[h.getName()], 4.0)

        vf.setPoseTarget(high_manip.Tstart[vf.getName()], 4.0)
        
        def after_task_complete():
            base.setActivationState(pyci.ActivationState.Enabled)
            return pelvis_center()

        return wait_tasks(hands + [vf], after_task_complete)

class pelvis_center:
    def __call__(self):
        polycenter = sum([t.getPoseReference()[0].translation for t in wheels])
        polycenter = polycenter / 4.0
        polycenter[0] -= 0.05
        T, _, _ = base.getPoseReference()
        T.translation[:2] = polycenter[:2] 
        base.setPoseTarget(T, 3.0)

        w = wheels[3]
        T, _, _ = w.getPoseReference()
        T.translation[0] += 0.25
        w.setPoseTarget(T, 2.0)

        w = wheels[0]
        T, _, _ = w.getPoseReference()
        T.translation[0] += 0.10
        w.setPoseTarget(T, 2.0)

        return wait_tasks([base] + wheels, lambda: coll_avoid_demo())

class goto:
    def __init__(self, task, tgt, time):

        if not isinstance(task, list):
            task = [task]
        
        if not isinstance(tgt, list):
            tgt = [tgt]

        if not isinstance(time, list):
            time = [time]

        for taski, tgti, timei in zip(task, tgt, time):
             taski.setPoseTarget(tgti, timei)

        self.tasks = task

    def __call__(self):
        if all([t.getTaskState() == pyci.State.Online for t in self.tasks]):
            return True

class sequence:
    def __init__(self, states, next_state):
        self.states = list(states)
        self.next_state = next_state
        self.active_state = states[0]()
    def __call__(self):
        done = self.active_state()
        if done is True:
            self.states.pop(0)
            if len(self.states) == 0:
                return self.next_state()
            self.active_state = self.states[0]()
            print('done, {} remaining..'.format(len(self.states)))

class coll_avoid_demo(sequence):
    def __init__(self):

        # change ci to basic stack
        global ci
        ci = ci_basic
        ci.reset(time)
        ci.update(time, dt)

        # relevant tasks
        lhand = ci.getTask('arm1_8')
        rhand = ci.getTask('arm2_8')
        com = ci.getTask('com')
        flwheel = ci.getTask('wheel_1')

        # list of poses
        states = list()

        # front polygon side
        wh_poses = [model.getPose('wheel_' + str(i+1)) for i in range(4)]
        tgt_pos = (wh_poses[0].translation + wh_poses[1].translation)/2.0
        T = Affine3(pos=tgt_pos)

        # l start post
        rstart = model.getPose('arm2_8')
        lstart = model.getPose('arm1_8')

        # old man watching construction site
        Tr = rstart.copy()
        Tr.translation[0] -= 1.0
        Tr.translation[1] -= 0.15

        Tl = lstart.copy()
        Tl.translation[0] -= 1.0
        Tl.translation[1] = 0.15

        # states.append(lambda Tr=Tr, Tl=Tl: goto([lhand, rhand], [Tl, Tr], [6.0, 6.0]))
        # states.append(lambda: wait_converged(lambda: True))

        # states.append(lambda Tr=rstart, Tl=lstart: goto([lhand, rhand], [Tl, Tr], [6.0, 6.0]))
        # states.append(lambda: wait_converged(lambda: True))

        # disable right, go with left
        def l_ee_off():
            lhand.setActivationState(pyci.ActivationState.Disabled)
            rhand.setActivationState(pyci.ActivationState.Enabled)
            return True

        def r_ee_off():
            rhand.setActivationState(pyci.ActivationState.Disabled)
            lhand.setActivationState(pyci.ActivationState.Enabled)
            return True

        def lr_ee_on():
            rhand.setActivationState(pyci.ActivationState.Enabled)
            lhand.setActivationState(pyci.ActivationState.Enabled)
            return True

        states.append(lambda: r_ee_off)
        states.append(lambda T=T: goto(lhand, T, 6.0))
        states.append(lambda: wait_time(1.0, lambda: True))
        states.append(lambda T=lstart: goto(lhand, T, 6.0))

        states.append(lambda: l_ee_off)
        states.append(lambda T=T: goto(rhand, T, 6.0))
        states.append(lambda: wait_time(1.0, lambda: True))
        states.append(lambda T=rstart: goto(rhand, T, 6.0))
        
        states.append(lambda: wait_converged(lambda: True))

        # extreme right hand motions
        T = rstart.copy()
        T.translation[0] += 1.0
        states.append(lambda T=T: goto(rhand, T, 6.0))
        states.append(lambda: wait_time(1.0, lambda: True))
        states.append(lambda T=rstart: goto(rhand, T, 8.0))

        T = rstart.copy()
        T.translation[1] += 1.5
        states.append(lambda T=T: goto(rhand, T, 6.0))
        states.append(lambda: wait_time(1.0, lambda: True))
        states.append(lambda T=rstart: goto(rhand, T, 8.0))

        T = rstart.copy()
        T.translation[2] += 1.0
        states.append(lambda T=T: goto(rhand, T, 6.0))
        states.append(lambda: wait_time(1.0, lambda: True))
        states.append(lambda T=rstart: goto(rhand, T, 8.0))

        T = rstart.copy()
        T.translation[0] -= 1.5
        states.append(lambda T=T: goto(rhand, T, 6.0))
        states.append(lambda: wait_time(1.0, lambda: True))
        states.append(lambda T=rstart: goto(rhand, T, 8.0))

        # com left-right
        states.append(lambda: wait_converged(lambda: True))
        states.append(lambda: lr_ee_on)

        comstart = Affine3(pos=model.getCOM())

        for _ in range(2):
            T = comstart.copy()
            T.translation[1] += 0.15
            states.append(lambda T=T: goto(com, T, 2.0))

            T = comstart.copy()
            T.translation[1] -= 0.15
            states.append(lambda T=T: goto(com, T, 2.0))

        states.append(lambda: wait_converged(lambda: True))

        # raise wheel
        whstart = wh_poses[0]
        T = whstart.copy()
        T.translation[2] += 0.45
        states.append(lambda T=T: goto(flwheel, T, 3.0))
        states.append(lambda: wait_converged(lambda: True))

        states.append(lambda T=whstart: goto(flwheel, T, 3.0))
        states.append(lambda T=comstart: goto(com, T, 3.0))
        states.append(lambda: wait_converged(lambda: True))

        




        sequence.__init__(self, states, lambda: wait_time(3.0, lambda: home()))


class home:
    def __call__(self):
        global ci
        ci = ci_car_frame
        ci.reset(time) 
        ci.update(time, dt)

        wh_pos = all_wheels(np.array([0.35, 0.35]))

        for ref, w in zip(wh_pos, wheels):
            T, _, _ = w.getPoseReference()
            T.translation[:2] = ref 
            w.setPoseTarget(T, 6.0)

        global done
        done = True

        return wait_tasks(wheels, lambda: noop())


class wait_tasks:
       def __init__(self, tasks, next_state):
           self.tasks = tasks
           self.next_state = next_state
       
       def __call__(self):
            if all([t.getTaskState() == pyci.State.Online for t in self.tasks]):
                return self.next_state()

class wait_time:
    def __init__(self, dt, next):
        self.next_state = next
        self.tend = time + dt
    def __call__(self):
        if time >= self.tend:
            return self.next_state()

class wait_converged:
    def __init__(self, next):
        self.next_state = next
    def __call__(self):
        if np.abs(model.getJointVelocity()).max() < 0.01:
            return self.next_state()

print('started looping..')
rate = rospy.Rate(1./dt * args.rate)
state = coll_avoid_demo()

while not done and not rospy.is_shutdown():

    # run state
    next_state = state()

    # run ik
    update_ik(ci, model, time, dt)
    rspub.publishTransforms('poses2')

    # send model state as reference
    qref = model.getJointPositionMap()
    dqref = model.eigenToMap(model.getJointVelocity())

    if not args.visual:
        robot.setPositionReference(qref)
        robot.setVelocityReference(dqref)
        robot.move()

    # switch state if required
    if next_state is not None and next_state != state:
        print('[{:.2f}] {} --> {}'.format(time, state.__class__.__name__, next_state.__class__.__name__))
        state = next_state

    # sleep
    rate.sleep()
    time += dt
    