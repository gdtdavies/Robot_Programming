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
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from cv_bridge import CvBridge, CvBridgeError
from tf2_geometry_msgs import do_transform_pose

class ImageProjection(Node):
    camera_model = None
    image_depth_ros = None

    visualisation = True
    # aspect ration between color and depth cameras
    # calculated as (color_horizontal_FOV/color_width) / (depth_horizontal_FOV/depth_width) from the dabai camera parameters
    color2depth_aspect = (71.0/640) / (67.9/400) # 0.65353461
    pothole_poses = PoseArray()

    def __init__(self):    
        super().__init__('image_projection')
        self.bridge = CvBridge()
        

        self.camera_info_sub = self.create_subscription(CameraInfo, '/limo/depth_camera_link/camera_info',
                                                self.camera_info_callback, 
                                                qos_profile=qos.qos_profile_sensor_data)
        
        self.object_location_pub = self.create_publisher(PoseArray, '/limo/object_location', 10)

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
        (numLabels, _, _, centroids) = out


        for i in range(1, numLabels): # skip the first component as it is the background
            (cX, cY) = centroids[i]
            cv2.circle(image_color, (int(cX), int(cY)), 4, (0, 255, 0), -1)

            # "map" from color to depth image
            depth_coords = (image_depth.shape[0]/2 + (cX - image_color.shape[0]/2)*self.color2depth_aspect, image_depth.shape[1]/2 + (cY - image_color.shape[1]/2)*self.color2depth_aspect)

            # check if the centroid is within the image boudaries
            if depth_coords[0] < 0 or depth_coords[0] >= image_depth.shape[0] or depth_coords[1] < 0 or depth_coords[1] >= image_depth.shape[1]:
                continue

            # get the depth reading at the centroid location
            depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])]

            # calculate object's 3d location in camera coords
            camera_coords = self.camera_model.projectPixelTo3dRay((cX, cY)) #project the image coords (x,y) into 3D ray in camera coords 
            camera_coords = [x/camera_coords[2] for x in camera_coords] # adjust the resulting vector so that z = 1
            camera_coords = [x*depth_value for x in camera_coords] # multiply the vector by depth

            #define a point in camera coordinates
            object_location = Pose()
            # object_location.frame_id = "depth_link"
            object_location.orientation.w = 1.0
            object_location.position.x = camera_coords[0]
            object_location.position.y = camera_coords[1]
            object_location.position.z = camera_coords[2]

            # print out the coordinates in the odom frame
            transform = self.get_tf_transform('depth_link', 'odom')
            p_camera = do_transform_pose(object_location, transform)

            if abs(p_camera.position.x) > 5 or abs(p_camera.position.y) > 5 or abs(p_camera.position.z) > 5:
                continue

            # print('Pothole coords: ', p_camera.position)

            # check if the array is empty
            if len(ImageProjection.pothole_poses.poses) == 0:
                ImageProjection.pothole_poses.poses.append(p_camera)
            else:
                found = False
                #check if the pothole is already in the list
                for pose in ImageProjection.pothole_poses.poses:
                    if abs(pose.position.x - p_camera.position.x) < 0.22 \
                    and abs(pose.position.y - p_camera.position.y) < 0.22:
                        found = True
                        # print('Pothole already in the list')
                        # print('Pothole coords: ', pose.position)
                        pose.position.x = (pose.position.x + p_camera.position.x) / 2
                        pose.position.y = (pose.position.y + p_camera.position.y) / 2
                        pose.position.z = (pose.position.z + p_camera.position.z) / 2
                        break
                if not found:
                    ImageProjection.pothole_poses.poses.append(p_camera)

            for i, pose in enumerate(ImageProjection.pothole_poses.poses):
                #if it finds two potholes with similar coordinates, it will combine them
                for j, pose2 in enumerate(ImageProjection.pothole_poses.poses):
                    if pose == pose2:
                        continue
                    if abs(pose.position.x - pose2.position.x) < 0.21 \
                    and abs(pose.position.y - pose2.position.y) < 0.21 :
                        pose.position.x = (pose.position.x + pose2.position.x) / 2
                        pose.position.y = (pose.position.y + pose2.position.y) / 2
                        pose.position.z = (pose.position.z + pose2.position.z) / 2
                        ImageProjection.pothole_poses.poses.remove(pose2)
                        print(f'Pothole {i+1} and {j+1} merged')
                        break
            
            # publish so we can see that in rviz
            # for pose in ImageProjection.pothole_poses.poses:
            #     self.object_location_pub.publish(pose)
            self.object_location_pub.publish(ImageProjection.pothole_poses)

        for i, pose in enumerate(ImageProjection.pothole_poses.poses):
            print('map coords', i+1, ':', pose.position)
        print('\n')

        if self.visualisation:
            cv2.imshow("image color", image_color)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    try:
        image_projection = ImageProjection()
        rclpy.spin(image_projection)
    except KeyboardInterrupt:
        image_projection.get_logger().info("Keyboard Interrupt ^C")
        print(len(ImageProjection.pothole_poses.poses))
    except Exception as e:
        image_projection.get_logger().error('Exception occurred: ' + str(e))
    finally:
        image_projection.destroy_node()
        rclpy.shutdown()    

if __name__ == '__main__':
    main()