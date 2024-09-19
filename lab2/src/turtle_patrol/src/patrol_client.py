#!/usr/bin/env python
import numpy as np
import rospy
from turtle_patrol.srv import Patrol  # Import service type
import sys

def patrol_client(turtle_name, vel, omega, x, y, theta):
    # Initialize the client node
    rospy.init_node(f'{turtle_name}_patrol_client')
    # Wait until patrol service is ready
    rospy.wait_for_service('/patrol')
    try:
        # Acquire service proxy
        patrol_proxy = rospy.ServiceProxy('/patrol', Patrol)
        rospy.loginfo(f'Command {turtle_name} to patrol with vel: {vel}, omega: {omega}')
        # Call patrol service via the proxy
        patrol_proxy(turtle_name, vel, omega, x, y, theta)
    except rospy.ServiceException as e:
        rospy.loginfo(e)

if __name__ == '__main__':

    if len(sys.argv) == 7:
        turtle_name = sys.argv[1]
        vel = float(sys.argv[2])
        omega = float(sys.argv[3])
        x = float(sys.argv[4])
        y = float(sys.argv[5])
        theta = float(sys.argv[6])
    else:
        print("Usage: patrol_client.py <turtle_name> <vel> <omega> <x> <y> <theta>")
        sys.exit(1)

    patrol_client(turtle_name, vel, omega, x, y, theta)