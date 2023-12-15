'''
Title: navigate.py
Author: George Davies
email: 27421139@students.lincoln.ac.uk

This file is used to control the robot's movement. It uses the laser scan data to determine the distance to the walls and then uses this information to determine the robot's velocity. 
If the robot is too close to the walls, it will reverse a little. If the robot is too close to the right wall, it will turn left. 
If the robot is too close to the left wall, it will turn right. If the robot is far enough from the walls, it will move forward.

Copied from src/pothole_finder/pothole_finder/navigate.py
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Navigator(Node):

    def __init__(self):
        super().__init__('navigator')
        # Subscriber to the laser scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.twist = Twist()

    ''' callback function for the laser scan data '''
    def scan_callback(self, msg):
        nb_samples = len(msg.ranges)

        # Get the minimum distance to the left, right and front walls
        left = min(msg.ranges[0:int(nb_samples/2)-1])
        right = min(msg.ranges[int(nb_samples/2)+1:nb_samples-1])
        front = min(msg.ranges[int(nb_samples/2)-50:int(nb_samples/2)+50])

        print(f"left: {round(left, 5)}, right: {round(right, 5)}, front: {round(front, 5)}")

        # Left/Right Twist velocities
        if right < 0.20:# If the robot is too close to the right wall, turn left
            self.twist.angular.z = -1.0
            print("turning left", end=" / ")
        elif left < 0.20:# If the robot is too close to the left wall, turn right
            self.twist.angular.z = 1.0
            print("turning right", end=" / ")
        else: # If the robot is far enough from the walls, don't turn
            self.twist.angular.z = 0.0
            print("not turning", end=" / ")

        # Forward/Reverse Twist velocities
        if front > 0.19: # If the robot is far enough from the walls in front, move forward
            self.twist.linear.x = 0.125
            print("moving forward")
        else: # If the robot is too close to the walls, reverse a little
            self.twist.linear.x = -0.05
            print("reversing")
        print()

        # Publish the velocity message to the cmd_vel topic
        self.publisher.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    navigator = Navigator()
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        print(" KeyboardInterrupt")
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
