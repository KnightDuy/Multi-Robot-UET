#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy
)


class WheelOdomPublisher(Node):
    def __init__(self):
        super().__init__('wheel_odom_publisher')

        # ===== PARAMETERS =====
        self.declare_parameter('robot_name', 'robot1')
        self.robot_name = self.get_parameter('robot_name').value
        self.prefix = self.robot_name + '_'

        # ===== ROBOT PARAMETERS =====
        self.wheel_radius = 0.034
        self.wheel_separation = 0.175

        # ===== STATE =====
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.last_time = None

        # ===== QoS =====
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        joint_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ===== PUBLISHER =====
        self.joint_pub = self.create_publisher(
            JointState,
            'joint_states',
            joint_qos
        )

        # ===== SUBSCRIBER =====
        self.odom_sub = self.create_subscription(
            Odometry,
            'odometry/filtered',
            self.odom_callback,
            odom_qos
        )

        self.get_logger().info(
            f'Wheel odom publisher started for {self.robot_name}'
        )

    def odom_callback(self, msg):
        current_time = self.get_clock().now()

        if self.last_time is None:
            self.last_time = current_time
            return

        # ===== DT =====
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # ===== VELOCITIES =====
        linear_vel = msg.twist.twist.linear.x
        angular_vel = msg.twist.twist.angular.z

        # ===== DIFF DRIVE =====
        left_wheel_vel = (
            linear_vel - angular_vel * self.wheel_separation / 2.0
        ) / self.wheel_radius

        right_wheel_vel = (
            linear_vel + angular_vel * self.wheel_separation / 2.0
        ) / self.wheel_radius

        # ===== INTEGRATE =====
        self.left_wheel_pos += left_wheel_vel * dt
        self.right_wheel_pos += right_wheel_vel * dt

        # ===== JOINT STATE =====
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = [
            self.prefix + 'left_wheel_joint',
            self.prefix + 'right_wheel_joint'
        ]
        joint_state.position = [
            self.left_wheel_pos,
            self.right_wheel_pos
        ]
        joint_state.velocity = [
            left_wheel_vel,
            right_wheel_vel
        ]

        self.joint_pub.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
