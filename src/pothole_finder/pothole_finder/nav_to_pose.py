'''
Title: nav_to_pose.py
Author: George Davies
email: 27421138@students.lincoln.ac.uk

acknowledgements: based on the code from LCAS/CMP9767_LIMO/assignment_template/assignment_template/example_nav_to_pose.py

'''
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import numpy as np

class navToPose():
    def __init__(self):
        self.navigator = BasicNavigator()
        self.navigator.declare_parameter('pose', [0.0, 0.0, 0.0])
        [x, y, theta] = self.navigator.get_parameter('pose').get_parameter_value().double_array_value

        initial_pose = self.get_initial_pose()
        self.navigator.setInitialPose(initial_pose)
        print("initial_pose set")

        self.navigator.waitUntilNav2Active()
        print("nav2 active")

        goal_pose = self.get_goal_pose(x, y, theta)
        self.navigator.goToPose(goal_pose)
        print("goal_pose set")

        self.info()
        self.finish()

        self.navigator.lifecycleShutdown()
        print("shutdown")


    def get_initial_pose(self):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'odom'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose = self.pose_from_xytheta(0.0, 0.0, 0.0)
        return initial_pose

    def get_goal_pose(self, x, y, theta):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'odom'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose = self.pose_from_xytheta(x, y, theta)
        return goal_pose

    def info(self):
        i = 0
        while not self.navigator.isTaskComplete():
            # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')


    def finish(self):
        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')


    def pose_from_xytheta(self, x, y, theta):
        # negative theta: turn clockwise
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        # because the pitch and roll are constant at 0, we can use a simpler conversion
        q = [np.cos(theta/2), 0.0, 0.0, np.sin(theta/2)]

        pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return pose
    
def main():
    rclpy.init()
    navigator = navToPose()
    rclpy.shutdown()


if __name__ == '__main__':
    main()