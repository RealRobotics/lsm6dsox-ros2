#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import MagneticField

from .lsm6dsox import *

# Time delay for each data read (in seconds)
time_delay = 0.01 #s


class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_lsm6dsox")
        self.get_logger().info("IMU Node has started")
        self.imu = drv_lsm6dsow(bus=1, adresse=LSM6DSOX_ADDR)

        self.imu_publisher = self.create_publisher(Imu, "imu/data_raw", 10)
        self.data_timer = self.create_timer(time_delay, self.publish_data)

    def publish_data(self):
        x_a, y_a, z_a = self.imu.read_accel()
        x_g, y_g, z_g = self.imu.read_gyro()
        msg = Imu()
        msg.header.frame_id = "imu"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.linear_acceleration.x = float (x_a) * SF_2G * 9.81
        msg.linear_acceleration.y = float (y_a) * SF_2G * 9.81
        msg.linear_acceleration.z = float (z_a) * SF_2G * 9.81
        msg.angular_velocity.x = float (x_g) * SF_200DPS * np.pi / 180
        msg.angular_velocity.y = float (y_g) * SF_200DPS * np.pi / 180
        msg.angular_velocity.z = float (z_g) * SF_200DPS * np.pi / 180
        self.imu_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()