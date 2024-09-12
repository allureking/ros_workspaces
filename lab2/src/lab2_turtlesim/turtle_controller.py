#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist  # Import the message type for velocity commands
import sys

def turtle_controller(turtle_name):
    # Publisher to send velocity commands to the turtle
    pub = rospy.Publisher(f'/{turtle_name}/cmd_vel', Twist, queue_size=10)

    # Create a rate object to control the loop rate
    rate = rospy.Rate(10)  # 10hz

    print(f"Controlling {turtle_name}. Use WASD keys to move, Q to quit.")

    while not rospy.is_shutdown():
        # Get user input for movement
        key = input("Enter command: ").lower()

        # Create a new Twist message to hold the velocity command
        twist = Twist()

        # Define the motion based on the key input
        if key == 'w':
            twist.linear.x = 2.0  # Move forward
        elif key == 's':
            twist.linear.x = -2.0  # Move backward
        elif key == 'a':
            twist.angular.z = 2.0  # Turn left
        elif key == 'd':
            twist.angular.z = -2.0  # Turn right
        elif key == 'q':
            print("Exiting...")
            break  # Exit the loop and stop the node
        else:
            print("Invalid key. Use WASD to move, Q to quit.")

        # Publish the velocity command to the turtle
        pub.publish(twist)

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    # Initialize the ROS node for controlling the turtle
    rospy.init_node('turtle_controller', anonymous=True)

    if len(sys.argv) != 2:
        print("Usage: turtle_controller.py <turtle_name>")
        sys.exit(1)

    # Get the turtle name from command line arguments
    turtle_name = sys.argv[1]

    try:
        turtle_controller(turtle_name)
    except rospy.ROSInterruptException:
        pass