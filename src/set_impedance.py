#!/usr/bin/env python3

from xbot_interface import xbot_interface as xbi
from cartesian_interface.pyci_all import get_xbot_config

import rospy 
import sys
import numpy 
import fnmatch

rospy.init_node('set_impedance')

cfg = get_xbot_config(prefix='xbotcore/')

joint = sys.argv[1]

tgt_imp = float(sys.argv[2])

t_trj = 5.0

robot = xbi.RobotInterface(cfg)

jid = robot.getDofIndex(joint)

if jid < 0:
    rospy.logerr(f'bad joint name {joint}')

robot.setControlMode(xbi.ControlMode.Stiffness())

for jn in robot.getEnabledJointNames():
    if fnmatch.fnmatch(jn, '*arm*'):
        print(f'set ctrl mode to idle for joint {jn}')
        robot.setControlMode({jn: xbi.ControlMode.Idle()})


k0 = robot.getStiffness()

if k0[jid] == 0:
    rospy.logerr('current impedance is zero')
    exit()

ratio = tgt_imp / k0[jid]

kf = k0 * ratio 

dt = 0.01

rate = rospy.Rate(hz=1./dt)

time = 0

print(f'will set impedance for {joint} from {k0[jid]} to {tgt_imp} in {t_trj} s; other joints will be scaled proportionally !')

while time <= t_trj:
    
    tau = time / t_trj
    alpha = tau 

    k = alpha*kf + (1-alpha)*k0 

    robot.setStiffness(k)
    robot.move()

    time += dt
    rate.sleep()

print('done')
