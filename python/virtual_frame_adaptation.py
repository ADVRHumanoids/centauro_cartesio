#!/usr/bin/env python

from cartesian_interface.pyci_all import *
import rospy
import math
import numpy as np


def main():

    # obtain ci ros client
    ci = pyci.CartesianInterfaceRos()

    # define time between waypoints
    time = 2.0

    # spin base
    ci.setControlMode('car_frame', pyci.ControlType.Velocity)
    vref = np.array([0.08, 0, 0, 0, 0, 0])
    ci.setVelocityReferenceAsync('car_frame', vref, -1.0)

    # pelvis down
    print 'Pelvis down..'
    ci.setTargetPose('pelvis', Affine3(pos=[0, 0, -0.05]), time)
    ci.waitReachCompleted('pelvis')
    base_pose_initial, _, _ = ci.getPoseReference('pelvis')

    # change base link to arms
    ci.setBaseLink('arm1_8', 'car_frame')
    ci.setBaseLink('arm2_8', 'car_frame')

    raw_input('ENTER to enlarge FR')

    # FR wheel to the right
    ci.setTargetPose('wheel_2', Affine3(pos=[0, -0.5, 0]), time, True)
    ci.waitReachCompleted('wheel_2')

    raw_input('ENTER to home FR and enlarge HR')

    # back to home
    ci.setTargetPose('wheel_2', Affine3(pos=[0, 0.5, 0]), time, True)
    ci.waitReachCompleted('wheel_2')

    # FR wheel to the right
    ci.setTargetPose('wheel_4', Affine3(pos=[0, -0.5, 0]), time, True)
    ci.waitReachCompleted('wheel_4')

    raw_input('ENTER to home HR')

    # back to home
    ci.setTargetPose('wheel_4', Affine3(pos=[0, 0.5, 0]), time, True)
    ci.waitReachCompleted('wheel_4')

    raw_input('ENTER to stop')

    ci.stopVelocityReferenceAsync('car_frame')


if __name__ == '__main__':
    rospy.init_node('virtual_frame_test')
    main()
