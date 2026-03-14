#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

_AXES2TUPLE = {'sxyz': (0, 0, 0, 0)}
_NEXT_AXIS = [1, 2, 0, 1]

def quaternion_from_euler(ai, aj, ak, axes='sxyz'):
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        firstaxis, parity, repetition, frame = axes

    i = firstaxis + 1
    j = _NEXT_AXIS[i + parity - 1] + 1
    k = _NEXT_AXIS[i - parity] + 1

    if frame:
        ai, ak = ak, ai
    if parity:
        aj = -aj

    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = np.empty((4,))
    if repetition:
        q[0] = cj * (cc - ss)
        q[i] = cj * (cs + sc)
        q[j] = sj * (cc + ss)
        q[k] = sj * (cs - sc)
    else:
        q[0] = cj * cc + sj * ss
        q[i] = cj * sc - sj * cs
        q[j] = cj * ss + sj * cc
        q[k] = cj * cs - sj * sc
    if parity:
        q[j] *= -1.0
    return q  # [w, x, y, z]

class OdomFromVel(Node):
    def __init__(self):
        super().__init__('odom_node')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.odom_pub = self.create_publisher(Odometry, '/robot2/odom', qos)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = None

        self.create_subscription(TwistStamped, '/robot2/vel_encoder/data', self.encoder_callback, qos)

        # Covariances
        self.pose_covariance = [0.01] + [0.0]*35
        self.pose_covariance[-1] = 0.02  # yaw
        self.twist_covariance = [0.1] + [0.0]*35
        self.twist_covariance[-1] = 0.1  # yaw rate

    def encoder_callback(self, msg: TwistStamped):
        current_time = msg.header.stamp

        if self.last_time is None:
           self.last_time = current_time
           return

        # Tính dt chính xác
        dt = ((current_time.sec - self.last_time.sec) +
            (current_time.nanosec - self.last_time.nanosec) * 1e-9)
        if dt <= 0.0:
            return
            
        self.last_time = current_time

        vx = msg.twist.linear.x
        vth = msg.twist.angular.z

        delta_x = vx * math.cos(self.th) * dt
        delta_y = vx * math.sin(self.th) * dt
        delta_th = vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        q = quaternion_from_euler(0.0, 0.0, self.th)

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "robot2_odom"
        odom.child_frame_id = "robot2_base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = q[0]
        odom.pose.pose.orientation.x = q[1]
        odom.pose.pose.orientation.y = q[2]
        odom.pose.pose.orientation.z = q[3]

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth

        odom.pose.covariance = self.pose_covariance
        odom.twist.covariance = self.twist_covariance

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = OdomFromVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
