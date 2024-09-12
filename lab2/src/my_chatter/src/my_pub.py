#!/usr/bin/env python

# Import the dependencies
import rospy
from my_chatter.msg import TimestampString
from std_msgs.msg import String


def talker():


    pub = rospy.Publisher("user_messages", TimestampString, queue_size=10)

    r = rospy.Rate(10) #10hz
    
    while not rospy.is_shutdown():
        user_input = input("Please enter a line of text and press <Enter>:")
        message = TimestampString()
        message.text = user_input
        message.timestamp = rospy.get_time()

        pub.publish(message)
        print(f"Sent: \"{message.text}\", Timestamp: {message.timestamp}")

        r.sleep()


if __name__ == '__main__':

    # Run this program as a new node in the ROS computation graph called /talker.
    rospy.init_node('talker', anonymous=True)

    # Check if the node has received a signal to shut down. If not, run the
    # talker method.
    try:
        talker()
    except rospy.ROSInterruptException: pass

