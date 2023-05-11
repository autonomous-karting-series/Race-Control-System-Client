"""ROS2 subscriber for testing RCS publisher to kart topic."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RCStest(Node):

    def __init__(self):
        super().__init__('rcs_test')

        self.subscription = self.create_subscription(String, "track_state",
                                                     self.track_callback, 10)
        self.subscription = self.create_subscription(String, "kart_state",
                                                     self.kart_callback, 10)

    def track_callback(self, msg):
        self.get_logger().info(f"track: {msg.data}")

    def kart_callback(self, msg):
        self.get_logger().info(f"kart: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = RCStest()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
