#!/usr/bin/env python

# Import the dependencies
import rospy
from my_chatter.msg import TimestampString

# Define the callback method which is called whenever this node receives a 
# message on its subscribed topic. The received message is passed as the first
# argument to callback().
def callback(message):
    recieved_time = rospy.get_time()
    print(f"Message: \"{message.text}\", Sent at: {message.timestamp}, Received at: {recieved_time}")


# Define the method which contains the node's main functionality
def listener():
    rospy.Subscriber("user_messages", TimestampString, callback)
    rospy.spin()


# Python's syntax for a main() method
if __name__ == '__main__':

    rospy.init_node('listener', anonymous=True)
    listener()
