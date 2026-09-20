#!/usr/bin/env python3
"""
mpu6050_node.py  --  reads the MPU-6050 on the Pi's I2C bus 1 (pins 3/5)
and publishes sensor_msgs/Imu on /imu/data_raw.

What it does NOT do: estimate orientation. The MPU-6050 has no magnetometer,
so its heading drifts forever. The EKF will only use the Z gyro (yaw RATE),
exactly as your roadmap addendum C says.

KEEP THE ROBOT STILL FOR THE FIRST 2 SECONDS: the node measures the gyro's
offset (bias) while stationary and subtracts it from every later reading.
"""

import math
import struct

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus          # from the apt package python3-smbus

# MPU-6050 registers (from the datasheet)
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B        # 14 bytes: accel xyz, temperature, gyro xyz
WHO_AM_I = 0x75

GYRO_LSB_PER_DPS = 131.0   # at +/-250 deg/s range
ACCEL_LSB_PER_G = 16384.0  # at +/-2 g range
G = 9.80665


class Mpu6050Node(Node):

    def __init__(self):
        super().__init__('mpu6050')
        self.bus_num = self.declare_parameter('i2c_bus', 1).value
        self.addr = self.declare_parameter('address', 0x68).value
        self.rate = self.declare_parameter('rate', 100.0).value
        self.frame_id = self.declare_parameter('frame_id', 'imu_link').value
        self.calib_samples = int(self.declare_parameter('calibration_seconds', 2.0).value
                                 * self.rate)
        # If turning the robot LEFT (anticlockwise seen from above) gives a
        # NEGATIVE angular_velocity.z, set this to -1.0 (the chip is upside down).
        self.z_sign = self.declare_parameter('gyro_z_sign', 1.0).value

        self.bus = SMBus(self.bus_num)
        who = self.bus.read_byte_data(self.addr, WHO_AM_I)
        self.get_logger().info(f'MPU-6050 WHO_AM_I = 0x{who:02X} (expect 0x68, '
                               'some clones give 0x70/0x72/0x98 - also fine)')

        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0x01)   # wake, PLL on gyro X
        self.bus.write_byte_data(self.addr, CONFIG, 0x03)       # 44 Hz low-pass filter
        self.bus.write_byte_data(self.addr, SMPLRT_DIV, 9)      # 1 kHz / (1+9) = 100 Hz
        self.bus.write_byte_data(self.addr, GYRO_CONFIG, 0x00)  # +/-250 deg/s
        self.bus.write_byte_data(self.addr, ACCEL_CONFIG, 0x00) # +/-2 g

        self.bias = [0.0, 0.0, 0.0]
        self.calib_sum = [0.0, 0.0, 0.0]
        self.calib_count = 0
        self.errors = 0

        self.pub = self.create_publisher(Imu, 'imu/data_raw', 20)
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info('Calibrating gyro bias - DO NOT MOVE THE ROBOT...')

    def read_raw(self):
        data = bytes(self.bus.read_i2c_block_data(self.addr, ACCEL_XOUT_H, 14))
        ax, ay, az, temp, gx, gy, gz = struct.unpack('>hhhhhhh', data)
        accel = [v / ACCEL_LSB_PER_G * G for v in (ax, ay, az)]
        gyro = [math.radians(v / GYRO_LSB_PER_DPS) for v in (gx, gy, gz)]
        temp_c = temp / 340.0 + 36.53
        return accel, gyro, temp_c

    def tick(self):
        try:
            accel, gyro, temp_c = self.read_raw()
        except OSError as err:
            self.errors += 1
            self.get_logger().warn(f'I2C read failed ({err}); total errors '
                                   f'{self.errors}', throttle_duration_sec=2.0)
            return

        # Phase 1: collect bias while standing still
        if self.calib_count < self.calib_samples:
            for i in range(3):
                self.calib_sum[i] += gyro[i]
            self.calib_count += 1
            if self.calib_count == self.calib_samples:
                self.bias = [s / self.calib_samples for s in self.calib_sum]
                self.get_logger().info(
                    'Gyro bias (rad/s): x=%.5f y=%.5f z=%.5f  | chip temp %.1f C'
                    % (*self.bias, temp_c))
                if max(abs(b) for b in self.bias) > 0.1:
                    self.get_logger().warn('Bias is large - was the robot moving '
                                           'during calibration? Restart the node.')
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.orientation_covariance[0] = -1.0          # "orientation not provided"
        msg.angular_velocity.x = gyro[0] - self.bias[0]
        msg.angular_velocity.y = gyro[1] - self.bias[1]
        msg.angular_velocity.z = (gyro[2] - self.bias[2]) * self.z_sign
        for i in (0, 4, 8):
            msg.angular_velocity_covariance[i] = 1e-4
            msg.linear_acceleration_covariance[i] = 0.04
        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Mpu6050Node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
