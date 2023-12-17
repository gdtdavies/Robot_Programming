'''
Title: collect_data.py
Author: George Davies
email: 27421138@students.lincoln.ac.uk

This script subscribes to the depth camera topic and saves the image as a jpg file.
'''

import rclpy
from rclpy.node import Node
from rclpy import qos

import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

class ImageCollector(Node):

    data_path = os.path.join(os.path.dirname(__file__), '../data/raw')

    def __init__(self):
        super().__init__("screenshot_collector")
        self.bridge = CvBridge()

        # Subscribe to the depth camera topic
        self.image_sub = self.create_subscription(
            Image, 
            "limo/depth_camera_link/image_raw", 
            self.image_callback,
            qos_profile=qos.qos_profile_sensor_data
        )

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Create the data directory if it doesn't exist
        if not os.path.exists(self.data_path):
            os.makedirs(self.data_path)

        i = 1
        # Check if there is already a screenshot.jpg file
        if not os.path.isfile(os.path.join(self.data_path, "screenshot.jpg")):
            path = os.path.join(self.data_path, "screenshot.jpg")
        else:
            # If there is, find the next available screenshot number
            while os.path.isfile(os.path.join(self.data_path, "screenshot") + str(i) + ".jpg"):
                i += 1
            path = os.path.join(self.data_path, "screenshot") + str(i) + ".jpg"
        
        # Save the image
        cv2.imwrite(path, cv_image)
        print(f"Screenshot saved as {path.split('/')[-1]}")

def main(args=None):
    rclpy.init(args=args)
    try:
        im_collect = ImageCollector()
        rclpy.spin(im_collect)
    except KeyboardInterrupt:
        print(" Keyboard Interrupt")
    except Exception as e:
        im_collect.get_logger().error('Exception occurred: ' + str(e))
    finally:
        im_collect.destroy_node()

    

if __name__ == "__main__":
    main()
