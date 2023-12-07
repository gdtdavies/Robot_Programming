import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Navigator(Node):

    def __init__(self):
        super().__init__('navigator')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.twist = Twist()

    def scan_callback(self, msg):
        # Process the laser scan data and calculate the desired robot velocity
        # based on the distance to the walls

        
        nb_samples = len(msg.ranges)

        left = min(msg.ranges[0:int(nb_samples/2)-1])
        right = min(msg.ranges[int(nb_samples/2)+1:nb_samples-1])
        front = min(msg.ranges[int(nb_samples/2)-50:int(nb_samples/2)+50])

        print(f"left: {left}, right: {right}, front: {front}")

        # Example: If the robot is too close to the right wall, turn left
        if right < 0.20:
            self.twist.angular.z = -1.0
            print("turning left")
        elif left < 0.20:
            self.twist.angular.z = 1.0
            print("turning right")
        # Example: If the robot is too close to the right wall, turn left
        else:
            self.twist.angular.z = 0.0
            print("not turning")

        # Example: If the robot is far enough from the walls in front, move forward
        if front > 0.19:
            self.twist.linear.x = 0.125
            print("moving forward")
        # Example: If the robot is too close to the walls, reverse a little
        else:
            self.twist.linear.x = -0.05
            # self.twist.angular.z *= -1.0 # invert the steering
            print("reversing")
        # else:
        #     self.twist.linear.x = 0.0
        #     print("not moving")

        self.publisher.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    navigator = Navigator()
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
