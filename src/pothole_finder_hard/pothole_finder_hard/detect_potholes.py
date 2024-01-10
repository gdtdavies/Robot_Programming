'''
Title: detect_potholes.py
Author: George Davies
email: 27421138@students.lincoln.ac.uk

acknowledgements: based on the code from LCAS/CMP9767_LIMO/uol_cmp9767m_tutorials/uol_cmp9767m_tutorials/image_projection_3.py

This node subscribes to the color image topic and the depth image topic
It detects the potholes in the color image and calculates their 3D coordinates
It publishes the coordinates in the map frame as a PoseArray message
'''
import rclpy
from rclpy.node import Node
from rclpy import qos

# OpenCV
import cv2

import os
import shutil
import math

# ROS libraries
import image_geometry
from tf2_ros import TransformListener, Buffer

# ROS2 message types
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped
from cv_bridge import CvBridge, CvBridgeError
from tf2_geometry_msgs import do_transform_pose

import matplotlib.pyplot as plt

# YOLOv8 by Ultralytics (object detection library)
from ultralytics import YOLO

# get the path of the current working directory
wd = os.path.dirname(os.path.realpath(__file__))

class PotholeDetector(Node):
    camera_model = None
    image_depth_ros = None
    transform = None

    C2D_ASPECT_RATIO = (71.0/640) / (67.9/400) # 0.65353461
    DISTANCE = 0.19 # distance in meters between two potholes for them to be considered the different potholes and not be merged

    pothole_poses = PoseArray()

    def __init__(self):
        super().__init__('pothole_detector')
        self.bridge = CvBridge()

        PotholeDetector.pothole_poses.header.frame_id = 'map'

        #subscriber to the camera info topic. it retreives the camera parameters
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/limo/depth_camera_link/camera_info',
            self.camera_info_callback, 
            qos_profile=qos.qos_profile_sensor_data
        )

        # publisher to the object_location topic. it publishes the location of the detected objects in the map
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

        # subscriber to the transform between the map frame and the depth camera frame found in get_transform.cpp
        self.tf_transform_sub = self.create_subscription(
            TransformStamped,
            '/depth_to_map_transform',
            self.tf_transform_callback,
            qos_profile=qos.qos_profile_sensor_data
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    #|#######################################################################|#
    #|############################<Misc Funtions>############################|#
    #|#######################################################################|#

    '''
    Params:
        pose_map: the pothole pose in the camera frame
    Returns:
        index: the index of the pothole in the array
    This function adds the pothole to the array of potholes
    If the pothole is already in the array, it merges the two potholes
    '''
    def add_pothole(self, pose_map):
        index = None # index of the pothole in the array
            # check if the array is empty
        if len(PotholeDetector.pothole_poses.poses) == 0:
            PotholeDetector.pothole_poses.poses.append(pose_map)
            index = 0
        else:
            found = False
            #check if the pothole is already in the list
            for pose in PotholeDetector.pothole_poses.poses:
                # check if the distance between the pothole and the pothole in the list is less than the radius of the pothole
                if abs(pose.position.x - pose_map.position.x) < self.DISTANCE \
                and abs(pose.position.y - pose_map.position.y) < self.DISTANCE:
                    found = True
                    # merge the two potholes
                    # the old pothole is weighted more than the new one as the old one is already an aggregate of multiple poses
                    pose.position.x = (pose.position.x*1.5 + pose_map.position.x*0.5) / 2
                    pose.position.y = (pose.position.y*1.5 + pose_map.position.y*0.5) / 2
                    index = PotholeDetector.pothole_poses.poses.index(pose)
                    break
            if not found:
                # there is no pothole in the list that is close enough to the current pothole
                # so add it to the list
                PotholeDetector.pothole_poses.poses.append(pose_map)
                index = len(PotholeDetector.pothole_poses.poses) - 1
        return index

    '''
    Params:
        index: the index of the pothole in the array
    This function merges the pothole with other potholes that are too close
    '''
    def merge_potholes(self, index):
        current_pose = PotholeDetector.pothole_poses.poses[index]
        for pose in PotholeDetector.pothole_poses.poses:
            if pose == current_pose:
                continue
            if abs(pose.position.x - current_pose.position.x) < self.DISTANCE \
            and abs(pose.position.y - current_pose.position.y) < self.DISTANCE :
                print(f'Pothole {index} and {PotholeDetector.pothole_poses.poses.index(pose)} merged')
                current_pose.position.x = (pose.position.x + current_pose.position.x) / 2
                current_pose.position.y = (pose.position.y + current_pose.position.y) / 2
                PotholeDetector.pothole_poses.poses.remove(pose)
                break

    '''
    Params:
        q: the quaternion to convert
    Returns:
        roll: the roll of the quaternion
        pitch: the pitch of the quaternion
        yaw: the yaw of the quaternion
    This function converts a quaternion to euler angles
    '''
    def quaternion_to_euler(self, q):
        # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_Angles_Conversion
        w, x, y, z = q.w, q.x, q.y, q.z
        roll = math.atan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
        pitch = math.asin(2*(w*y - z*x))
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
        return (roll, pitch, yaw)
        
    '''
    Params:
        pose: the detection pose in the map frame
        gt: the ground truth of the pothole
    Returns:
        the distance between the pothole and the ground truth
    This function calculates the distance between the pothole and the ground truth
    '''
    def distance(self, pose, gt):
        return ((pose.position.x - gt[1])**2 + (pose.position.y - gt[2])**2)**0.5

    '''
    Params:
        file: the file containing the ground truths
    Returns:
        out: a list of the ground truths
    This function reads the ground truths from the file and returns the x and y coordinates
    '''
    def get_ground_truths(self, file):
        out = []
        with open(file) as f:
            for line in f:
                if not line.startswith('*'):
                    continue
                line = line.split('|')
                pothole_nb = int(line[1])
                cx = float(line[2])
                cy = float(line[3])

                out.append([pothole_nb, cx, cy])
        return out

    '''
    Params:
        gts: the ground truths
    This function plots the positions of the ground truths and the detected potholes
    '''
    def plot_potholes(self, gts):
        fig, ax = plt.subplots()
        ax.set_aspect('equal')
        ax.set_xlim(-1.5, 1.5)
        ax.set_ylim(-1.2, 0.2)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.grid(True, which='both')
        ax.axhline(y=0, color='k')
        ax.axvline(x=0, color='k')
        ax.scatter([x[1] for x in gts], [x[2] for x in gts], c='r', label='ground truth')
        ax.scatter([pose.position.x for pose in PotholeDetector.pothole_poses.poses], [pose.position.y for pose in PotholeDetector.pothole_poses.poses], c='b', label='detected')
        ax.legend()
        plt.draw()
        plt.pause(2)
        plt.close(fig)


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

    '''callback function for the tf transform subscriber'''
    def tf_transform_callback(self, data):
        self.transform = data

    '''
    callback function for the color image subscriber
    it detects the potholes in the image and calculates their 3D coordinates
    it publishes the coordinates in the map frame
    '''
    def image_color_callback(self, data):
         # wait for camera_model, depth image, and the transform to arrive
        if self.camera_model is None:
            return
        if self.image_depth_ros is None:
            return
        if self.transform is None:
            return

        # covert images to open_cv
        try:
            image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "passthrough")
        except CvBridgeError as e:
            print(e)

        #----------------------------------------------------------------------
        #-YOLOv8---------------------------------------------------------------
        #----------------------------------------------------------------------

        pred_path = wd + '/../../../src/pothole_finder_hard/yolo/runs/detect/predict/'
        # delete the old predictions
        if os.path.exists(pred_path):
            shutil.rmtree(pred_path)

        # load the model creted by train_model.py
        model_path = wd + '/../../../src/pothole_finder_hard/yolo/runs/detect/tune2/weights/best.pt'
        model = YOLO(model_path)
        # detect the potholes in the image from the camera
        results = model(image_color, save=True, save_conf=True, save_txt=True, conf=0.5)

        # get the image and labels file
        img = cv2.imread(pred_path + '/image0.jpg')
        labels_files = pred_path + '/labels/image0.txt'

        # get the image height and width
        img_h, img_w = img.shape[0], img.shape[1]

        #----------------------------------------------------------------------
        #-pothole detection----------------------------------------------------
        #----------------------------------------------------------------------
        gts = self.get_ground_truths(wd + '/../../../ground_truths.txt')

        if os.path.exists(labels_files):
            with open(labels_files) as f:
                # each line in the labels file is a label for an object in the image
                for line in f:
                    label = line.split(' ')
                    class_id = int(label[0])
                    # the labels are in the format: class_id, cx, cy, w, h normalized to the image size
                    # so they need to be multiplied by the image size to get the actual coordinates
                    cx = float(label[1]) * img_w
                    cy = float(label[2]) * img_h
                    w = float(label[3]) * img_w
                    h = float(label[4]) * img_h


                    # draw a circle at the center of the pothole
                    cv2.circle(img, (int(cx), int(cy)), 4, (0, 255, 0), -1)

                    cy = img_h - cy # flip the y coordinate

                    # map the coordinates from the image to the depth image
                    depth_coords = ((cx - img_w/2) + image_depth.shape[1]/2,
                                    (img_h/2 - cy) + image_depth.shape[0]/2)

                    # get the depth value at the coordinates
                    depth_value = image_depth[int(depth_coords[1]), int(depth_coords[0])]

                    # if the depth value is too big or too small, it is probably a misreading
                    # the detections seem to be more accurate when the depth value is less than 1
                    if depth_value <= 0.1 or 1 <= depth_value:
                        continue
                    
                    # can use roll and pitch to verify that the calcuations are correct. They should be close to 0
                    (roll, pitch, yaw) = self.quaternion_to_euler(self.transform.transform.rotation)

                    # calculate object's 3d location in camera coords
                    #https://github.com/strawlab/vision_opencv/blob/master/image_geometry/src/image_geometry/cameramodels.py
                    # inspired by the function pixelTo3dRay in code from the link above
                    # the issue with that function is that 0,0 is the center of the image where it needed to be the bottom left corner
                    # potentially better if using trigonometry to scale the depth value
                    cam_x = (cx / self.camera_model.fx()) * (depth_value * abs(math.cos(yaw)))
                    cam_y = (cy / self.camera_model.fy()) * (depth_value * abs(math.sin(yaw)))
                    cam_z = 1.0

                    # define a point in camera coordinates
                    object_location = PoseStamped()
                    object_location.header.frame_id = 'depth_camera_link'
                    
                    object_location.pose.position.x = cam_x
                    object_location.pose.position.y = cam_y
                    object_location.pose.position.z = cam_z
                    object_location.pose.orientation.w = 1.0

                    # transform the point into the map frame
                    pose_map = do_transform_pose(object_location.pose, self.transform)


                    # reject the position if it is outside of the map boudaries or on the grass
                    if abs(pose_map.position.x) > 1.5 or pose_map.position.y < -1.3  or 0.2 < pose_map.position.y \
                    or 0.04 < pose_map.position.x and pose_map.position.x < 0.95 and -0.76 < pose_map.position.y and pose_map.position.y < -0.26 \
                    or -0.95 < pose_map.position.x and pose_map.position.x < -0.04 and -0.76 < pose_map.position.y and pose_map.position.y < -0.26:
                        continue

                    # add the pothole too the list and merge the pothole with other potholes that are too close
                    self.merge_potholes(self.add_pothole(pose_map))
                    # PotholeDetector.pothole_poses.poses.append(pose_map)


                # set the orientation of the potholes to be straight up and the z coordinate to be 0
                for pose in PotholeDetector.pothole_poses.poses:
                    pose.position.z = 0.0
                    pose.orientation.w = 0.707
                    pose.orientation.x = 0.0
                    pose.orientation.y = -0.707
                    pose.orientation.z = 0.0
        else:
            print('No potholes detected in the image')

        #----------------------------------------------------------------------
        #-Output---------------------------------------------------------------
        #----------------------------------------------------------------------   
         
        # self.plot_potholes(gts)     

        # publish so we can see that in rviz 
        self.object_location_pub.publish(PotholeDetector.pothole_poses)

        # print out the coordinates in the map frame
        print('----------------------------------------------------------------')
        for i, pose in enumerate(PotholeDetector.pothole_poses.poses):
            closest = [0, 99999, 99999]
            dist_to_gt = 99999
            for gt in gts[1:]:
                dist = self.distance(pose, gt)
                if self.distance(pose, closest) > self.distance(pose, gt):
                    closest = gt
                    dist_to_gt = dist
            
            print('pose {:2d} :\t x= {:.3f}\t|\ty= {:.3f}\t|\tclosest= {:2d}\t|\tdist= {:.2f}m'.format(i+1, pose.position.x, pose.position.y, closest[0], dist_to_gt))

        # show the image
        img = cv2.resize(img, (0,0), fx=0.5, fy=0.5)
        cv2.imshow('image', img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    pothole_detector = PotholeDetector()
    try:
        rclpy.spin(pothole_detector)
    except KeyboardInterrupt:
        print(" KeyboardInterrupt")
        print(f"found {len(PotholeDetector.pothole_poses.poses)} potholes")
    finally:
        pred_path = wd + '/../../../src/pothole_finder_hard/yolo/runs/detect/predict/'
        if os.path.exists(pred_path):
            shutil.rmtree(pred_path)
        pothole_detector.destroy_node()

if __name__ == '__main__':
    main()