#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState  # ROS message for joint states
from forward_kinematics import baxter_forward_kinematics_from_joint_state
import tf2_ros   # tf2 used to listen to transforms
import tf2_ros.tf2_ros
import geometry_msgs.msg

def joint_state_callback(msg, tfBuffer):
    """
    Callback function that is triggered when a new joint state message is received.
    It computes forward kinematics and compares the result with the transform 
    from the 'tf' system.
    """
    # Compute forward kinematics using the joint state message
    transformation_matrix = baxter_forward_kinematics_from_joint_state(msg)

    # Display the forward kinematics result
    rospy.loginfo("Forward Kinematics Transformation Matrix:\n%s", transformation_matrix)

    # Compare with the tf transform between 'base' and 'left_hand'
    try:
        
        # Lookup transform from base to left_hand
        # and then Extract translation and rotation from the transform

        trans = tfBuffer.lookup_transform('base', 'left_hand', rospy.Time())

        translation = trans.transform.translation
        rotation = trans.transform.rotation

        rospy.loginfo("TF Translation: [x: %f, y: %f, z: %f]", translation.x, translation.y, translation.z)
        rospy.loginfo("TF Rotation (quaternion): [x: %f, y: %f, z: %f, w: %f]", 
                      rotation.x, rotation.y, rotation.z, rotation.w)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        # Handle the exception if the transform could not be looked up
        rospy.logwarn("Could not get transform: %s", str(e))


def forward_kinematics_node():
    """
    Initializes the forward kinematics node, subscribes to /robot/joint_states topic,
    and listens to transforms between 'base' and 'left_hand'.
    """
    rospy.init_node('forward_kinematics_node', anonymous=True)

    # Create a buffer and a listener for tf
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    # Subscribe to the /robot/joint_states topic
    rospy.Subscriber("/robot/joint_states", JointState, joint_state_callback, tfBuffer)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    forward_kinematics_node()
