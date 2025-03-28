#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import Float32MultiArray
import serial
import re

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        # Publisher for acceleration data
        self.pub_accel = self.create_publisher(Float32MultiArray, 'mpu6050/acceleration', 10)
        # Publisher for gyroscope data
        self.pub_gyro = self.create_publisher(Float32MultiArray, 'mpu6050/gyroscope', 10)

        # Serial connection to Arduino (adjust the port and baud rate if necessary)
        self.ser = serial.Serial('/dev/ttyACM0', 115200)

        # Timer that calls the callback function every 0.1 seconds (10 Hz)
        self.timer = self.create_timer(0.1, self.read_serial_data)

    def read_serial_data(self):
        if self.ser.in_waiting > 0:
            # Read a line, decode it with latin-1 and remove extra spaces
            line = self.ser.readline().decode('latin-1').strip()
            self.get_logger().info(f"Received line: {line}")

            # Regex to extract acceleration data:
            # Example format: "Acceleration X: -2.62, Y: 8.15, Z: 4.75 m/s^2"
            accel_regex = r'Acceleration X:\s*([-\d.]+),\s*Y:\s*([-\d.]+),\s*Z:\s*([-\d.]+) m/s\^?2'
            # Regex to extract gyroscope data:
            # Example format: "Rotation X: -0.04, Y: 0.01, Z: -0.03 rad/s"
            gyro_regex = r'Rotation X:\s*([-\d.]+),\s*Y:\s*([-\d.]+),\s*Z:\s*([-\d.]+) rad/s'

            match_accel = re.search(accel_regex, line)
            match_gyro = re.search(gyro_regex, line)

            if match_accel and match_gyro:
                ax = float(match_accel.group(1))
                ay = float(match_accel.group(2))
                az = float(match_accel.group(3))

                gx = float(match_gyro.group(1))
                gy = float(match_gyro.group(2))
                gz = float(match_gyro.group(3))

                # Create and publish message for acceleration
                accel_msg = Float32MultiArray(data=[ax, ay, az])
                self.pub_accel.publish(accel_msg)

                # Create and publish message for gyroscope
                gyro_msg = Float32MultiArray(data=[gx, gy, gz])
                self.pub_gyro.publish(gyro_msg)

def main(args=None):
    rclpy.init(args=args)
    mpu6050_node = MPU6050Node()
    rclpy.spin(mpu6050_node)

    # Cleanup
    mpu6050_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
