#!/usr/bin/env python

import argparse
import rospy
import intera_interface
from intera_interface import CHECK_VERSION

def joint_position_control():
    """
    A node that takes joint angles as input from the user and moves the robot's arm to the desired position.
    """

    # Initialize the ROS node
    rospy.init_node('joint_position_angles')

    # Get limb instance for Sawyer (assuming right arm)
    limb = intera_interface.Limb('right')

    # Define joint names
    joint_names = limb.joint_names()

    # Ask the user for joint angles for each joint
    joint_angles = {}
    for joint in joint_names:
        angle = float(input(f"Enter angle for {joint}: "))
        joint_angles[joint] = angle

    # Set the joint positions
    limb.set_joint_positions(joint_angles)

    # Optionally, keep sending the command for a period to ensure execution
    rate = rospy.Rate(100)  # 100Hz refresh rate
    while not rospy.is_shutdown():
        limb.set_joint_positions(joint_angles)
        rate.sleep()

if __name__ == '__main__':
    try:
        joint_position_control()
    except rospy.ROSInterruptException:
        pass
