#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult, FollowWaypoints
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import TransformListener, Buffer

class waypointFollower(Node):

    def __init__(self):
        self.navigator = BasicNavigator()

        self.location_sub = self.create_subscription(PoseStamped, '/tf', self.follow_callback, 10)


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def follow_callback(self, msg):
        base_frame = msg.header.frame_id
        child_frame = msg.child_frame_id

        if base_frame != 'map' or child_frame != 'base_link':
            return

        initial_pose = msg

        self.navigator.setInitialPose(initial_pose)

        self.navigator.waitUntilNav2Active()
        self.navigator.changeMap('../../../maps/assignment_map.yaml')

        global_costmap = self.navigator.getGlobalCostmap()
        local_costmap = self.navigator.getLocalCostmap()

        goal_poses = []

        goal_pose1 = PoseStamped()
        goal_pose1.header.frame_id = 'map'
        goal_pose1.child_frame_id = 'base_link'
        goal_pose1.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose1.pose.position.x = 0.03
        goal_pose1.pose.position.y = -0.712
        goal_pose1.pose.orientation.w = 0.0
        goal_pose1.pose.orientation.z = 0.0

        # additional goals can be appended
        goal_pose2 = PoseStamped()
        goal_pose2.header.frame_id = 'map'
        goal_pose1.child_frame_id = 'base_link'
        goal_pose2.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose2.pose.position.x = 0.502
        goal_pose2.pose.position.y = -0.477
        goal_pose2.pose.orientation.w = 0.0
        goal_pose2.pose.orientation.z = 0.0
        
        goal_pose3 = PoseStamped()
        goal_pose3.header.frame_id = 'map'
        goal_pose1.child_frame_id = 'base_link'
        goal_pose3.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose3.pose.position.x = -0.013
        goal_pose3.pose.position.y = -0.608
        goal_pose3.pose.orientation.w = 0.0
        goal_pose3.pose.orientation.z = 0.0
        
        goal_poses.append(goal_pose1)
        goal_poses.append(goal_pose2)
        goal_poses.append(goal_pose3)

        


"""
Basic navigation demo to go to poses.
"""

def main():
    rclpy.init()

    navigator = BasicNavigator()
    # navigator = NavigateThroughPoses()

    # Set our demo's initial pose
    # find the initial pose by running `ros2 topic echo /amcl_pose`

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, navigator)
    try:
        # Get the transform at the specified time
        transform = tf_buffer.lookup_transform("base_link", "map", rclpy.time.Time())

        # # Extract pose information from the transform
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        # # Convert the transform to a Pose message
        initial_pose = PoseStamped()
        initial_pose.header = transform.header
        initial_pose.pose.position = translation
        initial_pose.pose.orientation = rotation

        # # Process further as needed
        print("Received Pose:")
        print("Position: x={}, y={}, z={}".format(translation.x, translation.y, translation.z))
        print("Orientation: x={}, y={}, z={}, w={}".format(rotation.x, rotation.y, rotation.z, rotation.w))

        # transform_stamped = tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

        # initial_pose = PoseStamped()
        # initial_pose.header.frame_id = 'map'
        # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        # initial_pose.pose.position.x = 0.0
        # initial_pose.pose.position.y = 0.0
        # initial_pose.pose.orientation.w = 0.0
        # initial_pose.pose.orientation.z = 0.0
        navigator.setInitialPose(initial_pose)
    except Exception as e:
        print(f"Error: {e}")
        exit(1)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    navigator.changeMap('../../../maps/assignment_map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    global_costmap = navigator.getGlobalCostmap()
    local_costmap = navigator.getLocalCostmap()

    # set our demo's goal poses to follow
    goal_poses = []

    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 0.03
    goal_pose1.pose.position.y = -0.712
    goal_pose1.pose.orientation.w = 0.0
    goal_pose1.pose.orientation.z = 0.0

    # additional goals can be appended
    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 0.502
    goal_pose2.pose.position.y = -0.477
    goal_pose2.pose.orientation.w = 0.0
    goal_pose2.pose.orientation.z = 0.0
    
    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = -0.013
    goal_pose3.pose.position.y = -0.608
    goal_pose3.pose.orientation.w = 0.0
    goal_pose3.pose.orientation.z = 0.0
    
    goal_poses.append(goal_pose1)
    goal_poses.append(goal_pose2)
    goal_poses.append(goal_pose3)

    # sanity check a valid path exists
    path = navigator.getPath(initial_pose, goal_pose1)

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        if i % 5 != 0:
            continue
        feedback = navigator.getFeedback()
        if feedback:
            print(
                'Executing current waypoint: '
                + str(feedback.current_waypoint + 1)
                + '/'
                + str(len(goal_poses))
            )
            now = navigator.get_clock().now()

            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=600.0):
                navigator.cancelTask()

            # Some follow waypoints request change to demo preemption
            if now - nav_start > Duration(seconds=35.0):
                goal_pose4 = PoseStamped()
                goal_pose4.header.frame_id = 'map'
                goal_pose4.header.stamp = now.to_msg()
                goal_pose4.pose.position.x = -1.13
                goal_pose4.pose.position.y = -0.99
                goal_pose4.pose.orientation.w = 0.0
                goal_pose4.pose.orientation.z = 0.0
                goal_poses = [goal_pose4]
                nav_start = now
                navigator.followWaypoints(goal_poses)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
