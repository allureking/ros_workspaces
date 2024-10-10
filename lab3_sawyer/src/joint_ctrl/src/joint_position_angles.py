#!/usr/bin/env python

import argparse
import rospy
import intera_interface
from intera_interface import CHECK_VERSION


def move_to_joint_positions(limb_side):
    """
    Move the Sawyer robot's limb to a specific set of joint positions.
    
    Parameters:
    - limb_side: The side of the robot's limb (e.g., 'right')
    """

    # Initialize the limb interface for the specified side
    limb = intera_interface.Limb(limb_side)

    # Get the joint names for the limb
    joints = limb.joint_names()

    # Prompt the user for joint angles
    joint_angles = {}
    print("Please enter the desired angles for the following joints:")

    # Take user input for each joint's desired angle
    for joint in joints:
        angle = float(input(f"Enter angle for {joint}: "))
        joint_angles[joint] = angle

    # Move to the specified joint positions
    print("Moving to specified joint positions...")

    # Create a rate object to control how fast the loop runs
    rate = rospy.Rate(100)  # 100 Hz

    # Set the joint positions until the robot reaches the desired positions
    while not rospy.is_shutdown():
        limb.set_joint_positions(joint_angles)
        rate.sleep()


def main():
    """
    Main function to initialize the node and run the joint position control.
    """

    # Get robot parameters
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()

    # Check if any limb is detected
    if not valid_limbs:
        rp.log_message("Cannot detect any limb parameters on this robot. Exiting.", "ERROR")
        return

    # Argument parser to take limb argument from command line
    parser = argparse.ArgumentParser(
        description="Control Sawyer robot by providing joint angles.",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "-l", "--limb", dest="limb", default=valid_limbs[0],
        choices=valid_limbs,
        help="Limb on which to run the joint position control example"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    # Initialize the ROS node
    print("Initializing node... ")
    rospy.init_node("sdk_joint_position_angles")

    # Enable the robot
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    # Define a shutdown function
    def clean_shutdown():
        print("\nExiting example.")

    # Register the shutdown function to be called on node exit
    rospy.on_shutdown(clean_shutdown)

    # Enable the robot and run the control function
    rospy.loginfo("Enabling robot...")
    rs.enable()

    # Call the joint position control function
    move_to_joint_positions(args.limb)

    print("Done.")


if __name__ == '__main__':
    main()
