import rclpy
from rclpy.node import Node
from rclpy import qos

import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

class ImageCollector(Node):

    data_path = os.path.join(os.path.dirname(__file__), '../data')

    def __init__(self):
        super().__init__("screenshot_collector")
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, 
            "limo/depth_camera_link/image_raw", 
            self.image_callback,
            qos_profile=qos.qos_profile_sensor_data
        )

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        i = 1
        if not os.path.exists(self.data_path):
            os.makedirs(self.data_path)

        if not os.path.isfile(os.path.join(self.data_path, "screenshot.jpg")):
            path = os.path.join(self.data_path, "screenshot.jpg")
        else:
            while os.path.isfile(os.path.join(self.data_path, "screenshot") + str(i) + ".jpg"):
                i += 1
            path = os.path.join(self.data_path, "screenshot") + str(i) + ".jpg"
        # Save the image
        cv2.imwrite(path, cv_image)
        print("Screenshot saved as %s", path.split("/")[-1])

def main(args=None):
    rclpy.init(args=args)
    try:
        im_collect = ImageCollector()
        rclpy.spin(im_collect)
    except KeyboardInterrupt:
        im_collect.get_logger().info("Keyboard Interrupt ^C")
    except Exception as e:
        im_collect.get_logger().error('Exception occurred: ' + str(e))
    finally:
        im_collect.destroy_node()

    

if __name__ == "__main__":
    main()
