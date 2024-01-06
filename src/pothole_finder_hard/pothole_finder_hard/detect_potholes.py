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

# ROS libraries
import image_geometry
from tf2_ros import TransformListener, Buffer

# ROS2 message types
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from tf2_geometry_msgs import do_transform_pose

import matplotlib.pyplot as plt
import numpy as np

# YOLOv8 by Ultralytics (object detection library)
from ultralytics import YOLO

wd = os.path.dirname(os.path.realpath(__file__))

class PotholeDetector(Node):
    camera_model = None
    image_depth_ros = None

    C2D_ASPECT_RATIO = (71.0/640) / (67.9/400) # 0.65353461
    DISTANCE = 0.19 # distance in meters between two potholes for them to be considered the different potholes and not be merged

    pothole_poses = PoseArray()

    def __init__(self):
        super().__init__('pothole_detector')
        self.bridge = CvBridge()

        PotholeDetector.pothole_poses.header.frame_id = 'map'

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/limo/depth_camera_link/camera_info',
            self.camera_info_callback, 
            qos_profile=qos.qos_profile_sensor_data
        )

        self.object_location_pub = self.create_publisher(
            PoseArray, 
            '/limo/object_location', 
            10
        )

        self.image_sub = self.create_subscription(
            Image, 
            '/limo/depth_camera_link/image_raw', 
            self.image_color_callback, 
            qos_profile=qos.qos_profile_sensor_data
        )

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

    # def get_tf_transform(self, target_frame, source_frame):
    #     try:
    #         transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
    #         return transform
    #     except Exception as e:
    #         self.get_logger().warning(f"Failed to lookup transform: {str(e)}")
    #         return None

    '''
    Params:
        posestamped: the pose to transform
        target_frame: the frame to transform to
    Returns:
        transformed: the new pose in the target frame
    This is a wrapper function to get the transformed pose to the target frame
    calls Buffer.transform() and returns the PoseStamped
    raises an exception if the transform failed
    '''
    def tf_transform(self, posestamped, target_frame):
        try:
            posestamped.header.stamp = rclpy.time.Time().to_msg()
            transformed = self.tf_buffer.transform(posestamped, target_frame)
            return transformed
        except Exception as e:
            self.get_logger().warning(f"Failed to transform pose: {str(e)}")
            return None

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

    def distance(self, pose, gt):
        return ((pose.position.x - gt[1])**2 + (pose.position.y - gt[2])**2)**0.5

    def get_ground_truths(self, file):
        out = []
        with open(file) as f:
            for line in f:
                # print(line)
                if not line.startswith('*'):
                    continue
                line = line.split('|')
                pothole_nb = int(line[1])
                cx = float(line[2])
                cy = float(line[3])

                out.append([pothole_nb, cx, cy])
        return out

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

    '''
    callback function for the color image subscriber
    it detects the potholes in the image and calculates their 3D coordinates
    it publishes the coordinates in the map frame
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
        model_path = wd + '/../../../src/pothole_finder_hard/yolo/runs/detect/tune/weights/best.pt'
        model = YOLO(model_path)
        # detect the potholes in the image from the camera
        results = model(image_color, save=True, save_conf=True, save_txt=True, conf=0.5)

        # get the image and labels file
        img = cv2.imread(pred_path + '/image0.jpg')
        labels_files = pred_path + '/labels/image0.txt'

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

                    # print(f'cx: {cx}, cy: {cy}, w: {w}, h: {h}')

                    # draw a circle at the center of the pothole
                    cv2.circle(img, (int(cx), int(cy)), 4, (0, 255, 0), -1)

                    cy = img_h - cy # flip the y coordinate

                    # map the coordinates from the image to the depth image
                    # depth_coords = (image_color.shape[0]/2 + (cx - image_depth.shape[0]/2) * self.C2D_ASPECT_RATIO,
                    #                 image_color.shape[1]/2 + (cy - image_depth.shape[1]/2) * self.C2D_ASPECT_RATIO)
                    depth_coords = ((cx - img_w/2) + image_depth.shape[1]/2,
                                    (img_h/2 - cy) + image_depth.shape[0]/2)
                    # depth_coords = (cx * self.C2D_ASPECT_RATIO, cy * (1/self.C2D_ASPECT_RATIO))
                    # print(f'depth_coords: {[int(x) for x in depth_coords]}')
                    # cv2.circle(image_depth, (int(depth_coords[0]), int(depth_coords[1])), 4, (0, 0, 255), -1)

                    # get the depth value at the coordinates
                    depth_value = image_depth[int(depth_coords[1]), int(depth_coords[0])]
                    # depth_value = image_depth[int(cx), int(cy)]

                    # if the depth value is too big, it is probably a misreading
                    # the detections seem to be more accurate when the depth value is less than 1
                    # print(f'depth_value: {depth_value}')
                    if depth_value >= 1:
                        continue

                    # calculate object's 3d location in camera coords
                    #https://github.com/strawlab/vision_opencv/blob/master/image_geometry/src/image_geometry/cameramodels.py
                    # potentially better if using trigonometry to scale to depth
                    cam_x = cx / self.camera_model.fx() * depth_value
                    cam_y = cy / self.camera_model.fy() * depth_value
                    cam_z = 1.0

                    # define a point in camera coordinates
                    object_location = PoseStamped()
                    object_location.header.frame_id = 'depth_camera_link'
                    
                    object_location.pose.position.x = cam_x
                    object_location.pose.position.y = cam_y
                    object_location.pose.position.z = cam_z
                    object_location.pose.orientation.w = 1.0

                    # transform the point into the map frame
                    pose_map = self.tf_transform(object_location, 'map')
                    # if the transform failed, skip the pothole
                    if pose_map is None:
                        continue
                    # get the pose from the PoseStamped
                    pose_map = pose_map.pose
                    # print(f'x: {pose_map.position.x}, y: {pose_map.position.y}')           

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
        # image_depth = cv2.resize(image_depth, (0,0), fx=0.5, fy=0.5)
        # cv2.imshow('depth', image_depth)
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