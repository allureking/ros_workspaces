#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
from intera_interface import gripper as robot_gripper

# Initialize the gripper
right_gripper = robot_gripper.Gripper('right_gripper')

# Define the recorded pick and place positions
pick_position = {
    "position": [0.752, -0.015, -0.161],
    "orientation": [0.077, 0.987, -0.112, 0.090]
}

place_position = {
    "position": [0.752, 0.255, -0.147],
    "orientation": [0.084, 0.996, -0.021, 0.035]
}

# The fuction to move the gripper to the desired object position
def move_to_pose(compute_ik, group, position, orientation):
    # Construct the IK request
    request = GetPositionIKRequest()
    request.ik_request.group_name = "right_arm"
    request.ik_request.ik_link_name = "right_gripper_tip"
    request.ik_request.pose_stamped.header.frame_id = "base"

    # Set the desired position and orientation for the end effector
    request.ik_request.pose_stamped.pose.position.x = position[0]
    request.ik_request.pose_stamped.pose.position.y = position[1]
    request.ik_request.pose_stamped.pose.position.z = position[2]
    request.ik_request.pose_stamped.pose.orientation.x = orientation[0]
    request.ik_request.pose_stamped.pose.orientation.y = orientation[1]
    request.ik_request.pose_stamped.pose.orientation.z = orientation[2]
    request.ik_request.pose_stamped.pose.orientation.w = orientation[3]
    
    try:
        # Send the request to the IK service
        response = compute_ik(request)
        
        # Check if the IK solution is valid
        if response.result_type[0] > 0:
            rospy.loginfo("IK Solution Found!")
            group.set_pose_target(request.ik_request.pose_stamped)
            plan = group.plan()
            
            # Confirm the plan in RViz
            user_input = input("Enter 'y' if the trajectory looks safe in RVIZ: ")
            if user_input.lower() == 'y':
                group.execute(plan[1])
            else:
                rospy.logwarn("Trajectory was not confirmed. Skipping execution.")
        else:
            rospy.logerr("No valid IK solution found.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)


def main():
    # Initialize ROS and set up the service and MoveIt commander
    rospy.wait_for_service('compute_ik')
    rospy.init_node('pick_and_place_demo')

    # Create the IK service proxy and MoveIt group commander
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    group = MoveGroupCommander("right_arm")

    # Initialize and calibrate the gripper
    rospy.loginfo("Calibrating gripper...")
    right_gripper.calibrate()
    rospy.sleep(2.0)

    # Move to the pick position
    rospy.loginfo("Moving to pick position...")
    move_to_pose(compute_ik, group, pick_position["position"], pick_position["orientation"])

    # Close the gripper to grasp the object
    rospy.loginfo("Closing gripper to pick up the object...")
    right_gripper.close()
    rospy.sleep(1.0)

    # Move to the place position
    rospy.loginfo("Moving to place position...")
    move_to_pose(compute_ik, group, place_position["position"], place_position["orientation"])

    # Open the gripper to release the object
    rospy.loginfo("Opening gripper to release the object...")
    right_gripper.open()
    rospy.sleep(1.0)

    rospy.loginfo("Pick and place task completed.")
    

# Python's syntax for a main() method
if __name__ == '__main__':
    main()
