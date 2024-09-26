#!/usr/bin/env python

import rospy
import tf2_ros
import sys
import tf.transformations  # To convert quaternion to RPY

def tf_echo():
    # Initialize the ROS node
    rospy.init_node('tf_echo_node', anonymous=True)
    
    # Check that exactly two arguments (target and source frames) are provided
    if len(sys.argv) != 3:
        rospy.logerr("Usage: tf_echo.py <target_frame> <source_frame>")
        sys.exit(1)

    # Get target and source frames from command line arguments
    target_frame = sys.argv[1]
    source_frame = sys.argv[2]

    # Create a buffer and listener for tf
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    # Main loop: look up the transform and print it periodically
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        try:
            # Query the transform from source_frame to target_frame
            trans = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time())
            
            # Extract translation and rotation (quaternion)
            translation = trans.transform.translation
            rotation = trans.transform.rotation
            
            # Print translation
            rospy.loginfo("Translation: [x: %f, y: %f, z: %f]", translation.x, translation.y, translation.z)
            
            # Print rotation in quaternion
            rospy.loginfo("Rotation (quaternion): [x: %f, y: %f, z: %f, w: %f]", 
                          rotation.x, rotation.y, rotation.z, rotation.w)
            
            # Convert quaternion to roll-pitch-yaw (RPY)
            quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
            rpy = tf.transformations.euler_from_quaternion(quaternion)
            
            # Print rotation in RPY
            rospy.loginfo("Rotation (radians): [roll: %f, pitch: %f, yaw: %f]", rpy[0], rpy[1], rpy[2])
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # Handle exceptions and warn if the transform is not available
            rospy.logwarn("Could not get transform: %s", str(e))
        
        # Sleep for a bit before querying again
        rate.sleep()

if __name__ == '__main__':
    tf_echo()
