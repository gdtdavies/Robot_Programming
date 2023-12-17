'''
Title: map_potholes.py
Author: George Davies
email: 27421138@students.lincoln.ac.uk

acknowledgements: based on the code from LCAS/CMP9767_LIMO/uol_cmp9767m_tutorials/uol_cmp9767m_tutorials/image_projection_3.py

This node subscribes to the color image topic and the depth image topic
It detects the potholes in the color image and calculates their 3D coordinates
It publishes the coordinates in the odom frame as a PoseArray message
'''

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
from geometry_msgs.msg import Pose, PoseArray
from cv_bridge import CvBridge, CvBridgeError
from tf2_geometry_msgs import do_transform_pose

class ImageProjection(Node):
    camera_model = None
    image_depth_ros = None

    visualisation = True
    # aspect ration between color and depth cameras
    # calculated as (color_horizontal_FOV/color_width) / (depth_horizontal_FOV/depth_width) from the dabai camera parameters
    C2D_ASPECT_RATIO = (71.0/640) / (67.9/400) # 0.65353461
    DISTANCE = 0.185 # distance in meters between two potholes for them to be considered the different potholes and not be merged

    # global array to keep the pothole poses over the entire run
    pothole_poses = PoseArray()

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


    #|#######################################################################|#
    #|############################<Misc Funtions>############################|#
    #|#######################################################################|#
    '''
    Params:
        target_frame: the frame to transform to
        source_frame: the frame to transform from
    Returns:
        transform: the transform between the two frames
    This is a wrapper function to get the transform between two frames
    calls Buffer.lookup_transform() and returns the transform
    raises an exception if the transform is not found
    '''
    def get_tf_transform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().warning(f"Failed to lookup transform: {str(e)}")
            return None
    
    '''
    Params:
        img: the image to be processed
        mask_lower: the lower bound of the mask
        mask_upper: the upper bound of the mask
    Returns:
        (numLabels, labels, stats, centroids) tuple
        numLabels: the number of blobs found
        labels: a matrix with the same size as the image where each pixel has a value of 1 or 0 depending if it is part of a blob or not
        stats: contrains the following information about each blob:
            cv2.CC_STAT_LEFT The leftmost (x) coordinate of the bounding box
            cv2.CC_STAT_TOP The topmost (y) coordinate of the bounding box 
            cv2.CC_STAT_WIDTH The horizontal size of the bounding box
            cv2.CC_STAT_HEIGHT The vertical size of the bounding box
            cv2.CC_STAT_AREA The total area (in pixels) of the connected component
        centroids: an array with the x and y coordinates of the centroid of each blob

    This function takes an image and returns the blobs in the image  
    '''
    def get_pohole_stats(self, img, mask_lower=(150, 100, 50), mask_upper=(255, 255, 255)):
        mask = cv2.inRange(img, mask_lower, mask_upper)
        out = cv2.connectedComponentsWithStats(mask , 4, cv2.CV_32S)
        (numLabels, labels, stats, centroids) = out
        return (numLabels, labels, stats, centroids)

    '''
    Params:
        p_camera: the pothole pose in the camera frame
    Returns:
        index: the index of the pothole in the array
    This function adds the pothole to the array of potholes
    If the pothole is already in the array, it merges the two potholes
    '''
    def add_pothole(self, p_camera):
        index = None # index of the pothole in the array
            # check if the array is empty
        if len(ImageProjection.pothole_poses.poses) == 0:
            ImageProjection.pothole_poses.poses.append(p_camera)
            index = 0
        else:
            found = False
            #check if the pothole is already in the list
            for pose in ImageProjection.pothole_poses.poses:
                # check if the distance between the pothole and the pothole in the list is less than the radius of the pothole
                if abs(pose.position.x - p_camera.position.x) < self.DISTANCE \
                and abs(pose.position.y - p_camera.position.y) < self.DISTANCE:
                    found = True
                    # merge the two potholes
                    # the old pothole is weighted more than the new one as the old one is already an aggregate of multiple poses
                    pose.position.x = (pose.position.x*1.5 + p_camera.position.x*0.5) / 2
                    pose.position.y = (pose.position.y*1.5 + p_camera.position.y*0.5) / 2
                    index = ImageProjection.pothole_poses.poses.index(pose)
                    break
            if not found:
                # there is no pothole in the list that is close enough to the current pothole
                # so add it to the list
                ImageProjection.pothole_poses.poses.append(p_camera)
                index = len(ImageProjection.pothole_poses.poses) - 1
        return index

    '''
    Params:
        index: the index of the pothole in the array
    This function merges the pothole with other potholes that are too close
    '''
    def merge_potholes(self, index):
        current_pose = ImageProjection.pothole_poses.poses[index]
        for pose in ImageProjection.pothole_poses.poses:
            if pose == current_pose:
                continue
            if abs(pose.position.x - current_pose.position.x) < self.DISTANCE \
            and abs(pose.position.y - current_pose.position.y) < self.DISTANCE :
                print(f'Pothole {index} and {ImageProjection.pothole_poses.poses.index(pose)} merged')
                current_pose.position.x = (pose.position.x + current_pose.position.x) / 2
                current_pose.position.y = (pose.position.y + current_pose.position.y) / 2
                ImageProjection.pothole_poses.poses.remove(pose)
                break

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

        #----------------------------------------------------------------------
        #-image processing-----------------------------------------------------
        #----------------------------------------------------------------------

        # detect a pink blob in the color image
        # the RGB of the pothole is around (215, 25, 195)
        # (numLabels, labels, stats, centroids) = self.get_pohole_stats(image_color, (150, 0, 150), (255, 100, 255))

        # the HSV of the pothole is around (305, 80, 50) Hue is 0-360, Saturation is 0-100, Value is 0-100
        image_hsv = cv2.cvtColor(image_color, cv2.COLOR_BGR2HSV)
        (numLabels, labels, stats, centroids) = self.get_pohole_stats(image_hsv)

        #----------------------------------------------------------------------
        #-pothole detection----------------------------------------------------
        #----------------------------------------------------------------------

        for i in range(1, numLabels): # skip the first component as it is the background
            (cX, cY) = centroids[i] # 0, 0 is the top left corner of the image
            
            # draw the centroid on the image
            cv2.circle(image_color, (int(cX), int(cY)), 4, (0, 255, 0), -1)
            
            # draw the bounding box on the image
            left, top = stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP]
            width, height = stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
            top_left, bottom_right = (left, top), (left + width, top + height)
            cv2.rectangle(image_color, top_left, bottom_right, (255, 255, 0), 1)

            # invert the y coordinate so that 0, 0 is the bottom left corner of the image
            cY = image_color.shape[0] - cY 

            # map the coordinates in the color image onto depth image
            depth_coords = (cX * self.C2D_ASPECT_RATIO, cY * (1/self.C2D_ASPECT_RATIO))

            # get the depth reading at the centroid location
            depth_value = image_depth[int(depth_coords[1]), int(depth_coords[0])]
            
            #if the depth value is too big, it is probably a misreading
            if depth_value > 5:
                continue

            # calculate object's 3d location in camera coords
            camera_coords = self.camera_model.projectPixelTo3dRay((cX, cY)) #project the image coords (x,y) into 3D ray in camera coords 
            camera_coords = [x*depth_value for x in camera_coords] # multiply the vector by depth

            #define a point in camera coordinates
            object_location = Pose()
            object_location.position.x = camera_coords[0]
            object_location.position.y = camera_coords[1]
            object_location.position.z = camera_coords[2]
            object_location.orientation.w = 1.0

            # get the transform from the depth camera link to the odom frame
            transform = self.get_tf_transform('odom', 'depth_camera_link')
            p_camera = do_transform_pose(object_location, transform)

            # reject the position if it is outside of the map boudaries
            if abs(p_camera.position.x) > 1.5 or p_camera.position.y < -1.3  or 0.2 < p_camera.position.y :
                continue

            # reject the position if it is on the grass
            if 0.04 < p_camera.position.x and p_camera.position.x < 0.95 and -0.76 < p_camera.position.y and p_camera.position.y < -0.26\
            or -0.95 < p_camera.position.x and p_camera.position.x < -0.04 and -0.76 < p_camera.position.y and p_camera.position.y < -0.26:
                continue

            # add the pothole too the list and merge the pothole with other potholes that are too close
            self.merge_potholes(self.add_pothole(p_camera))
        
        # set the orientation of the potholes to be straight up and the z coordinate to be 0
        for pose in ImageProjection.pothole_poses.poses:
            pose.position.z = 0.0
            pose.orientation.w = 0.707
            pose.orientation.x = 0.0
            pose.orientation.y = -0.707
            pose.orientation.z = 0.0
        
        #----------------------------------------------------------------------
        #-Output---------------------------------------------------------------
        #----------------------------------------------------------------------
        
        # publish so we can see that in rviz 
        self.object_location_pub.publish(ImageProjection.pothole_poses)

        # print out the coordinates in the odom frame
        for i, pose in enumerate(ImageProjection.pothole_poses.poses):
            print(f'pose {str(i+1).zfill(2)} : x= {round(pose.position.x, 5)}\t y= {round(pose.position.y, 5)}\t z = {round(pose.position.z, 5)}')

        # show the images
        if self.visualisation:
            image_color = cv2.resize(image_color, (0,0), fx=0.5, fy=0.5)
            cv2.imshow("image color", image_color)
            cv2.waitKey(1)

        print()

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
        # rclpy.shutdown() # this causes an error, says that it has already been called on the given context

if __name__ == '__main__':
    main()