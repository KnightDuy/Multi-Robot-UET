import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class FakeVelNode(Node):

    def __init__(self):
        super().__init__('fake_vel_node')

        self.publisher_ = self.create_publisher(
            TwistStamped,
            '/robot2/vel_encoder/data',
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        self.get_logger().info("Fake vel encoder node started")

    def timer_callback(self):

        msg = TwistStamped()

        # timestamp rất quan trọng vì odom node dùng nó để tính dt
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        # fake velocity
        msg.twist.linear.x = 0.5
        msg.twist.angular.z = 0.1

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = FakeVelNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()