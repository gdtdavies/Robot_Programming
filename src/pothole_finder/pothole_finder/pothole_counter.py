import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray

class PotholeCounter(Node):

    def __init__(self):
        super().__init__('pothole_counter')
        self.subscription = self.create_subscription(
            PoseArray,
            '/limo/object_location',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)

    pothole_counter = PotholeCounter()

    rclpy.spin(pothole_counter)

    pothole_counter.destroy_node()
    rclpy.shutdown()