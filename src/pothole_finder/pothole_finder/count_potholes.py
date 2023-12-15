'''
Title: count_potholes.py
Author: George Davies
email: 27421138@students.lincoln.ac.uk

This file is used to get the count of potholes in the environment. 
It subscribes to the object_location topic and gets the length of the PoseArray.
This file was created mainly to verify that the publishing in map_potholes was working correctly
'''

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray

class PotholeCounter(Node):

    def __init__(self):
        super().__init__('pothole_counter')
        self.subscription = self.create_subscription(
            PoseArray,
            '/limo/object_location',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        nb_poses = len(msg.poses)
        print(f"{nb_poses} potholes have been detected")

def main(args=None):
    rclpy.init(args=args)
    try:
        pothole_counter = PotholeCounter()
        rclpy.spin(pothole_counter)
    except KeyboardInterrupt:
        print(" KeyboardInterrupt")
    finally:
        pothole_counter.destroy_node()

if __name__ == '__main__':
    main()