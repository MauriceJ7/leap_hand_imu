#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from leap_imu.msg import IMUData
import serial
import re

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')
        self.publisher = self.create_publisher(IMUData, 'mpu6050/data', 10)
        
        # Öffne die serielle Schnittstelle zum Arduino
        self.ser = serial.Serial('/dev/ttyACM0', 115200)
        
        # Timer, der alle 0,01 Sekunden (100 Hz) die Callback-Funktion aufruft
        self.timer = self.create_timer(0.01, self.read_serial_data)

    def read_serial_data(self):
        if self.ser.in_waiting > 0:
            # Lese eine Zeile, dekodiere sie mit latin-1 und entferne überflüssige Leerzeichen
            line = self.ser.readline().decode('latin-1').strip()
            self.get_logger().info(f"Received line: {line}")

            # Regex zum Extrahieren der Beschleunigungsdaten
            # Erwartetes Format: "Acceleration X: -2.62, Y: 8.15, Z: 4.75 m/s^2"
            accel_regex = r'Acceleration X:\s*([-\d.]+),\s*Y:\s*([-\d.]+),\s*Z:\s*([-\d.]+) m/s\^?2'
            # Regex zum Extrahieren der Rotationsdaten
            # Erwartetes Format: "Rotation X: -0.04, Y: 0.01, Z: -0.03 rad/s"
            gyro_regex  = r'Rotation X:\s*([-\d.]+),\s*Y:\s*([-\d.]+),\s*Z:\s*([-\d.]+) rad/s'
            
            match_accel = re.search(accel_regex, line)
            match_gyro  = re.search(gyro_regex, line)
            
            if match_accel and match_gyro:
                try:
                    ax = float(match_accel.group(1))
                    ay = float(match_accel.group(2))
                    az = float(match_accel.group(3))
                    
                    gx = float(match_gyro.group(1))
                    gy = float(match_gyro.group(2))
                    gz = float(match_gyro.group(3))
                except ValueError as e:
                    self.get_logger().error(f"Error converting values: {e}")
                    return

                # Erstelle eine Instanz der custom IMUData-Nachricht
                imu_msg = IMUData()
                imu_msg.acceleration = Vector3(x=ax, y=ay, z=az)
                imu_msg.rotation = Vector3(x=gx, y=gy, z=gz)

                # Publiziere die Nachricht
                self.publisher.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
