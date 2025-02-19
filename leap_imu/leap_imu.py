#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import serial
import re

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')
        
        # Create publishers for acceleration and gyroscope data
        self.pub_accel = self.create_publisher(Int16MultiArray, 'mpu6050/acceleration', 10)
        self.pub_gyro = self.create_publisher(Int16MultiArray, 'mpu6050/gyroscope', 10)

        # Open the serial port (adjust the port and baud rate as needed)
        self.ser = serial.Serial('/dev/ttyACM0', 115200)  # Change to your Arduino's port

        # Create a timer to read data at a fixed rate
        self.timer = self.create_timer(0.1, self.read_serial_data)  # 10 Hz

    def read_serial_data(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            # Use regex to extract acceleration and gyroscope data
            match = re.match(r'A:([-0-9]+),([-0-9]+),([-0-9]+) G:([-0-9]+),([-0-9]+),([-0-9]+)', line)
            if match:
                ax = int(match.group(1))
                ay = int(match.group(2))
                az = int(match.group(3))
                gx = int(match.group(4))
                gy = int(match.group(5))
                gz = int(match.group(6))

                # Publish acceleration data
                accel_msg = Int16MultiArray(data=[ax, ay, az])
                self.pub_accel.publish(accel_msg)

                # Publish gyroscope data
                gyro_msg = Int16MultiArray(data=[gx, gy, gz])
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
