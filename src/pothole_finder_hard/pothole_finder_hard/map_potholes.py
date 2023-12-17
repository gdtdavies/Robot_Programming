'''
Title: map_potholes.py
Author: George Davies
email: 27421138@students.lincoln.ac.uk

This node subscribes to the color image topic and the depth image topic
It detects the potholes in the color image and calculates their 3D coordinates
It publishes the coordinates in the odom frame as a PoseArray message

The potholes detected are from the potholes.world file rather than the potholes_simple.world file
'''

import rclpy
from rclpy.node import Node
from rclpy import qos

# OpenCV
import cv2

# ROS libraries
import image_geometry
from tf2_ros import TransformListener, Buffer

# ROS2 message types
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge, CvBridgeError
from tf2_geometry_msgs import do_transform_pose

class ImageProjection(Node):

    def __init__(self):
        super().__init__('image_projection')
        self.bridge = CvBridge()

        ImageProjection.pothole_poses.header.frame_id = 'odom'

        #subscriber to the camera info topic. it retreives the camera parameters
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/limo/depth_camera_link/camera_info',
            self.camera_info_callback, 
            qos_profile=qos.qos_profile_sensor_data
        )
        #publisher to the object_location topic. it publishes the location of the detected objects in the map
        # as a PoseArray message
        self.object_location_pub = self.create_publisher(
            PoseArray, 
            '/limo/object_location', 
            10
        )

        # subscriber to the color image topic. it retrieves the color image from the camera and performs the
        # detection of the potholes in its callback function
        self.image_sub = self.create_subscription(
            Image, 
            '/limo/depth_camera_link/image_raw', 
            self.image_color_callback, 
            qos_profile=qos.qos_profile_sensor_data
        )
        
        # subscriber to the depth image topic. it retrieves the depth image from the camera so it can be used
        # to calculate the 3D coordinates of the detected potholes
        self.image_sub = self.create_subscription(
            Image, 
            '/limo/depth_camera_link/depth/image_raw', 
            self.image_depth_callback, 
            qos_profile=qos.qos_profile_sensor_data
        )
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


    def detect_potholes(self, image_color):
        pass
        # detect the potholes in the image using yolov3
        


    #|#######################################################################|#
    #|##########################<Callback Funtions>##########################|#
    #|#######################################################################|#

    '''callback function for the camera info subscriber'''
    def camera_info_callback(self, data):
        if not self.camera_model:
            self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)

    '''callback function for the depth image subscriber'''
    def image_depth_callback(self, data):
        self.image_depth_ros = data

    '''
    callback function for the color image subscriber
    it detects the potholes in the image and calculates their 3D coordinates
    it publishes the coordinates in the odom frame
    '''
    def image_color_callback(self, data):
        # wait for camera_model and depth image to arrive
        if self.camera_model is None:
            return

        if self.image_depth_ros is None:
            return

        # covert images to open_cv
        try:
            image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
        except CvBridgeError as e:
            print(e)

        # detect potholes in the image
        pothole_poses = self.detect_potholes(image_color)

def main(args=None):
    rclpy.init(args=args)
    try:
        image_projection = ImageProjection()
        rclpy.spin(image_projection)
    except KeyboardInterrupt:
        print(f'\n{len(ImageProjection.pothole_poses.poses)} potholes detected')
        image_projection.get_logger().info("Keyboard Interrupt ^C")
    except Exception as e:
        image_projection.get_logger().error('Exception occurred: ' + str(e))
    finally:
        image_projection.destroy_node()

if __name__ == '__main__':
    main()

