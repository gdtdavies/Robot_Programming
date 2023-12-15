'''
Title: mask_potholes.py
Author: George Davies
email: 27421138@students.lincoln.ac.uk

This file is used to find potholes in the image data from the limo camera. 
It uses the cv_bridge package to convert the image data from the ROS message to a format that can be used by OpenCV.
'''
import rclpy
from rclpy.node import Node
from rclpy import qos

import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from time import sleep


class PotholeFinder(Node):

    graphical = True

    def __init__(self):
        super().__init__('find_pothole')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, 
            "/limo/depth_camera_link/image_raw",
            self.image_callback,
            qos_profile=qos.qos_profile_sensor_data
        ) 

    '''
    Params:
        image: the image to be masked
        name: the name of the image
        lo_range: the lower range of the mask
        hi_range: the upper range of the mask
    Returns:
        numLabels: the number of componants found
        labels: a matrix with the same size as the image where each pixel has a value of 1 or 0 depending if it is part of a componant or not
        stats: contrains the following information about each componant:
            cv2.CC_STAT_LEFT The leftmost (x) coordinate of the bounding box
            cv2.CC_STAT_TOP The topmost (y) coordinate of the bounding box 
            cv2.CC_STAT_WIDTH The horizontal size of the bounding box
            cv2.CC_STAT_HEIGHT The vertical size of the bounding box
            cv2.CC_STAT_AREA The total area (in pixels) of the connected component
        centroids: an array with the x and y coordinates of the centroid of each componant
    '''
    def mask(self, image, name, lo_range, hi_range):
        mask = cv2.inRange(image, lo_range, hi_range)
        out = cv2.connectedComponentsWithStats(mask , 4, cv2.CV_32S)
        (numLabels, labels, stats, centroids) = out
        if self.graphical:
            for i in range(1, numLabels):

                print("[INFO] examining component {}/{}".format(i + 1, numLabels))
                
                # extract the connected component statistics and centroid for
                # the current label
                x = stats[i, cv2.CC_STAT_LEFT]
                y = stats[i, cv2.CC_STAT_TOP]
                w = stats[i, cv2.CC_STAT_WIDTH]
                h = stats[i, cv2.CC_STAT_HEIGHT]
                (cX, cY) = centroids[i]

                # draw the bounding box and centroid for the current label
                output = image.copy()
                cv2.rectangle(output, (x, y), (x + w, y + h), (0, 255, 0), 3)
                cv2.circle(output, (int(cX), int(cY)), 4, (0, 0, 255), -1)

                # construct a mask for the current label and display it
                componentMask = (labels == i).astype("uint8") * 255

                # show our output image and connected component mask
                cv2.imshow(name, output)
                cv2.imshow("Connected Component", componentMask)
                sleep(1)
                cv2.waitKey(1)

        return numLabels, labels, stats, centroids


    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        cv_image = cv2.resize(cv_image, None, fx=1.0, fy=1.0, interpolation = cv2.INTER_CUBIC)
            
        numPotholes, pixels, stats, centroids = self.mask(cv_image, 'potholes', (150, 0, 150), (255, 100, 255))

def main(args=None):
    rclpy.init(args=args)
    try:
        image_converter = PotholeFinder()
        rclpy.spin(image_converter)
    except KeyboardInterrupt:
        print(" KeyboardInterrupt")
    finally:
        image_converter.destroy_node()

if __name__ == '__main__':
    main()
