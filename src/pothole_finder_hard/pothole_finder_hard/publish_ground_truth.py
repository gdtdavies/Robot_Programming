import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose

import os

wd = os.path.dirname(os.path.realpath(__file__))

class GroundTruthPublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_publisher')

        self.publisher_ = self.create_publisher(PoseArray, '/ground_truth_poses', 10)
        self.timer_ = self.create_timer(1.0, self.publish_poses)

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

    def publish_poses(self):
        poses = PoseArray()
        poses.header.frame_id = 'map'
        gts = self.get_ground_truths(wd + '/../../../ground_truths.txt')
        for gt in gts:
            pose = Pose()
            pose.position.x = gt[1]
            pose.position.y = gt[2]
            pose.position.z = 0.0
            pose.orientation.w = 0.707
            pose.orientation.x = 0.0
            pose.orientation.y = -0.707
            pose.orientation.z = 0.0
            poses.poses.append(pose)

        self.publisher_.publish(poses)
        print('Published %d ground truth poses' % len(poses.poses))

def main(args=None):
    rclpy.init(args=args)
    ground_truth_publisher = GroundTruthPublisher()
    rclpy.spin(ground_truth_publisher)
    ground_truth_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
