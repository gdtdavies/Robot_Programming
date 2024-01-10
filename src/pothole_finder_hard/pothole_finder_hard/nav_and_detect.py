'''
Title: nav_and_detect.py
Author: George Davies
email: 27421138@students.lincoln.ac.uk


This script runs the navigation and detection nodes in sequencially in a loop.
'''
import rclpy

import os
import sys
import time
import shutil
import asyncio

wd = os.path.dirname( __file__ )
sys.path.append( wd )
import detect_potholes
import navigate

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('nav_and_detect')

    navigator = navigate.Navigator()
    detector = detect_potholes.PotholeDetector()

    try:
        future = asyncio.Future()
        while rclpy.ok():
            # Run the detection node for 10 seconds
            rclpy.spin_until_future_complete(detector, future, timeout_sec=10.0)
            time.sleep(5)
            # Run the navigation node for 5 seconds
            rclpy.spin_until_future_complete(navigator, future, timeout_sec=5.0)
            time.sleep(3)
    except KeyboardInterrupt:
        print(" KeyboardInterrupt")
    finally:
        pred_path = os.path.dirname(os.path.realpath(__file__)) + '/../../../src/pothole_finder_hard/yolo/runs/detect/predict/'
        if os.path.exists(pred_path):
            shutil.rmtree(pred_path)
        node.destroy_node()
        navigator.destroy_node()
        detector.destroy_node()


if __name__=='__main__':
    main()

