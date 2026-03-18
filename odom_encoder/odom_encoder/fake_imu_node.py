#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from rclpy.qos import QoSProfile, ReliabilityPolicy
from collections import deque


class ImuMagFilter(Node):

    def __init__(self):
        super().__init__('imu_mag_optimized_filter')

        # Parameters
        self.declare_parameter('window_size', 3)
        self.declare_parameter('alpha', 0.25)
        self.declare_parameter('mag_alpha', 0.25)
        self.declare_parameter('bias_alpha', 0.0003)
        self.declare_parameter('enable_gyro_bias_estimation', True)
        self.declare_parameter('gravity_mps2', 9.81)
        self.declare_parameter('stationary_gyro_threshold', 0.12)
        self.declare_parameter('stationary_accel_tolerance', 1.2)

        self.N = self.get_parameter('window_size').value
        self.alpha = self.get_parameter('alpha').value
        self.mag_alpha = self.get_parameter('mag_alpha').value
        self.bias_alpha = self.get_parameter('bias_alpha').value
        self.enable_gyro_bias_estimation = self.get_parameter('enable_gyro_bias_estimation').value
        self.gravity_mps2 = self.get_parameter('gravity_mps2').value
        self.stationary_gyro_threshold = self.get_parameter('stationary_gyro_threshold').value
        self.stationary_accel_tolerance = self.get_parameter('stationary_accel_tolerance').value

        # Queues for moving average
        self.acc = [deque(maxlen=self.N), deque(maxlen=self.N), deque(maxlen=self.N)]
        self.gyro = [deque(maxlen=self.N), deque(maxlen=self.N), deque(maxlen=self.N)]

        # Previous filtered outputs
        self.acc_prev = [0.0, 0.0, 0.0]
        self.gyro_prev = [0.0, 0.0, 0.0]
        self.mag_prev = [0.0, 0.0, 0.0]

        # Gyro bias
        self.gyro_bias = [0.0, 0.0, 0.0]

        # QoS
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Sub
        self.create_subscription(Imu, '/robot2/imu/data', self.imu_callback, qos)
        self.create_subscription(MagneticField, '/robot2/mag/data', self.mag_callback, qos)

        # Pub
        self.pub_imu = self.create_publisher(Imu, '/robot2/imu/filtered', qos)
        self.pub_mag = self.create_publisher(MagneticField, '/robot2/mag/filtered', qos)

        self.get_logger().info("Optimized IMU + MAG filter running")

    # --- fast moving average ---
    def moving_avg(self, buf, value):
        buf.append(value)
        return sum(buf) / len(buf)

    # --- Low-pass filter ---
    def lpf(self, prev, current, a):
        return prev + a * (current - prev)

    # --- Gyro bias update ---
    def update_bias(self, idx, raw, is_stationary):
        b = self.gyro_bias[idx]
        if self.enable_gyro_bias_estimation and is_stationary:
            b = b + self.bias_alpha * (raw - b)
            self.gyro_bias[idx] = b
        return raw - b

    # ========== IMU ==========
    def imu_callback(self, msg):
        out = Imu()
        out.header = msg.header

        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z

        # Detect stationary periods for safe gyro bias learning.
        # Updating bias while robot is rotating quickly will corrupt angular velocity.
        gyro_norm = (gx * gx + gy * gy + gz * gz) ** 0.5
        acc_norm = (ax * ax + ay * ay + az * az) ** 0.5
        is_stationary = (
            gyro_norm < self.stationary_gyro_threshold and
            abs(acc_norm - self.gravity_mps2) < self.stationary_accel_tolerance
        )

        # --- ACC ---
        acc_values = [ax, ay, az]
        for i in range(3):
            avg = self.moving_avg(self.acc[i], acc_values[i])
            self.acc_prev[i] = self.lpf(self.acc_prev[i], avg, self.alpha)

        out.linear_acceleration.x = self.acc_prev[0]
        out.linear_acceleration.y = self.acc_prev[1]
        out.linear_acceleration.z = self.acc_prev[2]

        # --- GYRO ---
        gyro_raw = [gx, gy, gz]
        for i in range(3):
            corrected = self.update_bias(i, gyro_raw[i], is_stationary)
            avg = self.moving_avg(self.gyro[i], corrected)
            self.gyro_prev[i] = self.lpf(self.gyro_prev[i], avg, self.alpha)

        out.angular_velocity.x = self.gyro_prev[0]
        out.angular_velocity.y = self.gyro_prev[1]
        out.angular_velocity.z = self.gyro_prev[2]

        # giữ orientation nguyên
        out.orientation = msg.orientation

        self.pub_imu.publish(out)

    # ========== MAG ==========
    def mag_callback(self, msg):
        out = MagneticField()
        out.header = msg.header

        mx, my, mz = msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z

        self.mag_prev[0] = self.lpf(self.mag_prev[0],  mx, self.mag_alpha)
        self.mag_prev[1] = self.lpf(self.mag_prev[1], -my, self.mag_alpha)
        self.mag_prev[2] = self.lpf(self.mag_prev[2], -mz, self.mag_alpha)

        out.magnetic_field.x = self.mag_prev[0]
        out.magnetic_field.y = self.mag_prev[1]
        out.magnetic_field.z = self.mag_prev[2]

        self.pub_mag.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuMagFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

