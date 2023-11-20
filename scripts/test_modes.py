#! /usr/bin/env python

import sys

import actionlib
import rospy
from controller_manager_msgs.srv import SwitchController
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from kortex_hardware.srv import *

if __name__ == '__main__':
    # get command line arguments for control mode
    mode = sys.argv[1]
    topic = ""
    if mode == "position":
        topic = "/position_controller/command"
    elif mode == "velocity":
        topic = "/velocity_controller/command"
    elif mode == "effort":
        topic = "/effort_controller/command"

    rospy.wait_for_service('controller_manager/switch_controller')
    rospy.wait_for_service('set_control_mode')
    switch_controller = rospy.ServiceProxy(
        'controller_manager/switch_controller', SwitchController)
    mode_change = rospy.ServiceProxy('set_control_mode', ModeService)
    try:
        if mode == "position":
            resp = switch_controller(['position_controller'], ['velocity_controller',
                                                               'effort_controller'], 1, 0, 5)
        elif mode == "effort":
            resp = switch_controller(['effort_controller'], ['velocity_controller',
                                                             'position_controller'], 1, 0, 5)
        elif mode == "velocity":
            resp = switch_controller(['velocity_controller'], ['position_controller',
                                                               'effort_controller'], 1, 0, 5)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

    try:
        resp1 = mode_change(mode)
        print("Mode change response: ", resp1)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
    
    if mode == "stop":
        sys.exit(0)

    try:
        rospy.init_node('jointgroup_test_py')
        cmd_pub = rospy.Publisher(topic, Float64MultiArray,
                queue_size=1) if mode == "position" else rospy.Publisher(topic,
                        JointTrajectory, queue_size=1)

        while not rospy.is_shutdown():
            # wait for message from joint_states
            msg = rospy.wait_for_message("/joint_states", JointState)
            cmd = Float64MultiArray() if mode == "position" else JointTrajectory()
            if mode != "position":
                cmd.joint_names = list(msg.name)[1:]
            positions = list(msg.position)[1:]
            velocities = [0.0 for i in range(len(positions))]
            efforts = [0.0 for i in range(len(positions))]
            print("Current position: ", positions)
            val = input("Command: ")
            dof = input("DOF: ")
            if mode != "position":
                timeout = input("Timeout: ")
            positions[int(dof) - 1] = float(val)
            if mode == "position":
                cmd.data = positions
            elif mode == "velocity" or mode == "effort":
                point = JointTrajectoryPoint()
                point.positions = positions
                point.velocities = velocities
                point.accelerations = efforts
                point.time_from_start = rospy.Duration(float(timeout))
                cmd.points = [point]
            cmd_pub.publish(cmd)
            input("Press Enter to continue...")

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
