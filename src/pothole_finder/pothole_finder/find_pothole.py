import rclpy
from rclpy.node import Node
from rclpy import qos
import cv2
from cv2 import namedWindow, cvtColor, imshow, inRange

from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2GRAY, waitKey
from cv2 import blur, Canny, resize, INTER_CUBIC
from cv2 import connectedComponentsWithStats
from numpy import mean
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from time import sleep


class PotholeFinder(Node):

    graphical = True

    def __init__(self):
        super().__init__('find_pothole')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, 
                                                    "/limo/depth_camera_link/image_raw",
                                                    self.image_callback,
                                                    qos_profile=qos.qos_profile_sensor_data) # Set QoS Profile

    def mask(self, image, name, lo_range, hi_range):
        mask = inRange(image, lo_range, hi_range)
        out = connectedComponentsWithStats(mask , 4, cv2.CV_32S)
        (numLabels, labels, stats, centroids) = out
        if self.graphical:
            for i in range(0, numLabels):
                # the first component is the background
                if i == 0:
                    continue

                print("[INFO] examining component {}/{}".format(i + 1, numLabels))
                
                x = stats[i, cv2.CC_STAT_LEFT]
                y = stats[i, cv2.CC_STAT_TOP]
                w = stats[i, cv2.CC_STAT_WIDTH]
                h = stats[i, cv2.CC_STAT_HEIGHT]
                area = stats[i, cv2.CC_STAT_AREA]
                (cX, cY) = centroids[i]

                output = image.copy()
                cv2.rectangle(output, (x, y), (x + w, y + h), (0, 255, 0), 3)
                cv2.circle(output, (int(cX), int(cY)), 4, (0, 0, 255), -1)

                componentMask = (labels == i).astype("uint8") * 255
                # show our output image and connected component mask
                
                
                imshow(name, output)
                imshow("Connected Component", componentMask)
                sleep(1)
                waitKey(1)

        return numLabels, labels, stats, centroids


    def image_callback(self, data):

        try:
            #convert the image to opencv format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        cv_image = resize(cv_image, None, fx=1.0, fy=1.0, interpolation = INTER_CUBIC)
            
        # imshow("Image window", cv_image)

        numPotholes, pixels, stats, centroids = self.mask(cv_image, 'potholes', (150, 0, 150), (255, 100, 255))
        #_, _, _, _ = self.mask(cv_image, 'lines', (0, 50, 200), (100, 255, 255))


        waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_converter = PotholeFinder()
    rclpy.spin(image_converter)

    image_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
