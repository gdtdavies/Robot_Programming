# Python libs
import rclpy
from rclpy.node import Node
from rclpy import qos

# OpenCV
import cv2

# ROS libraries
import image_geometry
from tf2_ros import Buffer, TransformListener

# ROS Messages
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from tf2_geometry_msgs import do_transform_pose

class ImageProjection(Node):
    camera_model = None
    image_depth_ros = None

    visualisation = True
    # aspect ration between color and depth cameras
    # calculated as (color_horizontal_FOV/color_width) / (depth_horizontal_FOV/depth_width) from the dabai camera parameters
    color2depth_aspect = (71.0/640) / (67.9/400)

    def __init__(self):    
        super().__init__('image_projection')
        self.bridge = CvBridge()

        self.camera_info_sub = self.create_subscription(CameraInfo, '/limo/depth_camera_link/camera_info',
                                                self.camera_info_callback, 
                                                qos_profile=qos.qos_profile_sensor_data)
        
        self.object_location_pub = self.create_publisher(PoseStamped, '/limo/object_location', 10)

        self.image_sub = self.create_subscription(Image, '/limo/depth_camera_link/image_raw', 
                                                  self.image_color_callback, qos_profile=qos.qos_profile_sensor_data)
        
        self.image_sub = self.create_subscription(Image, '/limo/depth_camera_link/depth/image_raw', 
                                                  self.image_depth_callback, qos_profile=qos.qos_profile_sensor_data)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


    def get_tf_transform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().warning(f"Failed to lookup transform: {str(e)}")
            return None

    def camera_info_callback(self, data):
        if not self.camera_model:
            self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)

    def image_depth_callback(self, data):
        self.image_depth_ros = data

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


        # detect a color blob in the color image
        # provide the right values, or even better do it in HSV
        image_mask = cv2.inRange(image_color, (150, 0, 150), (255, 100, 255))

        out = cv2.connectedComponentsWithStats(image_mask , 4, cv2.CV_32S)
        (numLabels, labels, stats, centroids) = out

        # print(numLabels)
        for i in range(1, numLabels): # skip the first component as it is the background
            (cX, cY) = centroids[i]
            cv2.circle(image_color, (int(cX), int(cY)), 4, (0, 255, 0), -1)

                    # "map" from color to depth image
            depth_coords = (image_depth.shape[0]/2 + (cX - image_color.shape[0]/2)*self.color2depth_aspect, image_depth.shape[1]/2 + (cY - image_color.shape[1]/2)*self.color2depth_aspect)

            if cX >= image_depth.shape[0] or cY >= image_depth.shape[1]:
                continue
            if depth_coords[0] >= image_depth.shape[0] or depth_coords[1] >= image_depth.shape[1]:
                continue


            # get the depth reading at the centroid location
            depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])] # you might need to do some boundary checking first!
            # depth_value = image_depth[int(cX), int(cY)] # you might need to do some boundary checking first!

            # calculate object's 3d location in camera coords
            camera_coords = self.camera_model.projectPixelTo3dRay((cX, cY)) #project the image coords (x,y) into 3D ray in camera coords 
            camera_coords = [x/camera_coords[2] for x in camera_coords] # adjust the resulting vector so that z = 1
            camera_coords = [x*depth_value for x in camera_coords] # multiply the vector by depth



            #define a point in camera coordinates
            object_location = PoseStamped()
            object_location.header.frame_id = "depth_link"
            object_location.pose.orientation.w = 1.0
            object_location.pose.position.x = camera_coords[0]
            object_location.pose.position.y = camera_coords[1]
            object_location.pose.position.z = camera_coords[2]

            # publish so we can see that in rviz
            self.object_location_pub.publish(object_location)        

            # print out the coordinates in the odom frame
            transform = self.get_tf_transform('depth_link', 'map')
            p_camera = do_transform_pose(object_location.pose, transform)

            print('map coords', i, ':', p_camera.position)

        if self.visualisation:
            #resize and adjust for visualisation
            # image_color = cv2.resize(image_color, (0,0), fx=0.5, fy=0.5)
            # image_depth *= 1.0/10.0 # scale for visualisation (max range 10.0 m)

            cv2.imshow("image color", image_color)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_projection = ImageProjection()
    rclpy.spin(image_projection)
    image_projection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# This Python script is for a ROS (Robot Operating System) node that subscribes to image and camera info topics, and 
# projects a point in 3D space to pixel coordinates in the image.
# Here's a breakdown of the code:

# 1. Import necessary libraries: ROS libraries, OpenCV for image processing, and sensor_msgs for ROS message types.

# 2. Define a class ImageProjection that inherits from Node. This class represents the ROS node.

# 3. In the __init__ method, initialize the node, create a CvBridge object for converting between ROS image messages 
# and OpenCV images, and create subscriptions to the image and camera info topics.

# 4. get_tf_transform is a method that tries to get the transformation between a target frame and a source frame. If it 
# fails, it logs a warning and returns None.

# 5. camera_info_callback is the method that gets called when a camera info message is received. It initializes the 
# camera model if it hasn't been initialized yet, and updates the camera model with the received data.

# 6. image_depth_callback is the method that gets called when a depth image message is received. It stores the received 
# image for later use.

# 7. image_color_callback is the method that gets called when a color image message is received. It waits for the camera 
# model and depth image to be available, then converts the ROS image messages to OpenCV images.

# The cursor is currently on line 0, which is the first line of the file.