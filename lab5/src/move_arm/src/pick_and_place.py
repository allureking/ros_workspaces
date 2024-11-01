#!/usr/bin/env python
import rospy
import numpy as np
import sys
# from numpy import linalg
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
from intera_interface import gripper as robot_gripper


def main():
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('pick_and_place_task')
    
    # Initialize the gripper
    right_gripper = robot_gripper.Gripper('right_gripper')
    print('Calibrating...')
    right_gripper.calibrate()
    rospy.sleep(2.0)

    # Create the function used to call the IK service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    while not rospy.is_shutdown():
        input('Press [ Enter ] to start the pick and place task: ')

        # =====================
        # PICK POSITION
        # =====================
        # Construct the IK request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"
        link = "right_gripper_tip"  # Ensure this is the correct link name
        request.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"

        # Set the pick position and orientation from recorded values
        request.ik_request.pose_stamped.pose.position.x = 0.752
        request.ik_request.pose_stamped.pose.position.y = -0.015
        request.ik_request.pose_stamped.pose.position.z = -0.161
        request.ik_request.pose_stamped.pose.orientation.x = 0.077
        request.ik_request.pose_stamped.pose.orientation.y = 0.987
        request.ik_request.pose_stamped.pose.orientation.z = -0.112
        request.ik_request.pose_stamped.pose.orientation.w = 0.090
        # request.ik_request.pose_stamped.pose.orientation.x = 0.0
        # request.ik_request.pose_stamped.pose.orientation.y = 1.0
        # request.ik_request.pose_stamped.pose.orientation.z = 0.0
        # request.ik_request.pose_stamped.pose.orientation.w = 0.0

        try:
            # Send the request to the IK service
            response = compute_ik(request)
            print(response)
            group = MoveGroupCommander("right_arm")

            # Set target pose for the arm
            group.set_pose_target(request.ik_request.pose_stamped)

            # Plan and visualize the IK solution
            plan = group.plan()
            user_input = input("Enter 'y' if the trajectory looks safe in RVIZ: ")
            
            if user_input == 'y':
                # Execute the pick motion
                group.execute(plan[1])
                rospy.sleep(1.0)

                # Close the gripper to pick up the object
                print('Closing gripper...')
                right_gripper.close()
                rospy.sleep(1.0)
            
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        # =====================
        # PLACE POSITION
        # =====================
        # Construct the IK request for the place position
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"
        request.ik_request.ik_link_name = link
        request.ik_request.pose_stamped.header.frame_id = "base"

        # Set the place position and orientation from recorded values
        request.ik_request.pose_stamped.pose.position.x = 0.752
        request.ik_request.pose_stamped.pose.position.y = 0.255
        request.ik_request.pose_stamped.pose.position.z = -0.147
        request.ik_request.pose_stamped.pose.orientation.x = 0.084
        request.ik_request.pose_stamped.pose.orientation.y = 0.996
        request.ik_request.pose_stamped.pose.orientation.z = -0.021
        request.ik_request.pose_stamped.pose.orientation.w = 0.035
        # request.ik_request.pose_stamped.pose.orientation.x = 0.0
        # request.ik_request.pose_stamped.pose.orientation.y = 1.0
        # request.ik_request.pose_stamped.pose.orientation.z = 0.0
        # request.ik_request.pose_stamped.pose.orientation.w = 0.0

        try:
            # Send the request to the IK service
            response = compute_ik(request)
            print(response)
            # group = MoveGroupCommander("right_arm")
            group.set_pose_target(request.ik_request.pose_stamped)

            # Plan and visualize the IK solution for placing
            plan = group.plan()
            user_input = input("Enter 'y' if the trajectory looks safe in RVIZ: ")

            if user_input == 'y':
                # Execute the place motion
                group.execute(plan[1])
                rospy.sleep(1.0)

                # Open the gripper to release the object
                print('Opening gripper...')
                right_gripper.open()
                rospy.sleep(1.0)
            
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

# Main method
if __name__ == '__main__':
    main()
