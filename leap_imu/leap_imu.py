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

        # Create publishers for acceleration and gyroscope data
        self.pub_accel = self.create_publisher(Float32MultiArray, 'mpu6050/acceleration', 10)
        self.pub_gyro = self.create_publisher(Float32MultiArray, 'mpu6050/gyroscope', 10)

        # Open the serial port (adjust the port and baud rate as needed)
        self.ser = serial.Serial('/dev/ttyACM0', 115200)  # Change to your Arduino's port

        # Create a timer to read data at a fixed rate
        self.timer = self.create_timer(0.01, self.read_serial_data)  # 100 Hz

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import re

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        # Publisher für Beschleunigungsdaten
        self.pub_accel = self.create_publisher(Float32MultiArray, 'mpu6050/acceleration', 10)
        # Publisher für Gyroskopdaten
        self.pub_gyro = self.create_publisher(Float32MultiArray, 'mpu6050/gyroscope', 10)

        # Serielle Verbindung zum Arduino (Anpassen des Ports und der Baudrate falls nötig)
        self.ser = serial.Serial('/dev/ttyACM0', 115200)

        # Timer, der alle 0,1 Sekunden (10 Hz) die Callback-Funktion aufruft
        self.timer = self.create_timer(0.1, self.read_serial_data)

    def read_serial_data(self):
        if self.ser.in_waiting > 0:
            # Lese eine Zeile, dekodiere sie mit latin-1 und entferne überflüssige Leerzeichen
            line = self.ser.readline().decode('latin-1').strip()
            self.get_logger().info(f"Empfangene Zeile: {line}")

            # Regex zum Extrahieren der Beschleunigungsdaten:
            # Beispiel-Format: "Acceleration X: -2.62, Y: 8.15, Z: 4.75 m/s^2"
            accel_regex = r'Acceleration X:\s*([-\d.]+),\s*Y:\s*([-\d.]+),\s*Z:\s*([-\d.]+) m/s\^?2'
            # Regex zum Extrahieren der Gyroskopdaten:
            # Beispiel-Format: "Rotation X: -0.04, Y: 0.01, Z: -0.03 rad/s"
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

                # Erstellen und Publizieren der Nachricht für Beschleunigung
                accel_msg = Float32MultiArray(data=[ax, ay, az])
                self.pub_accel.publish(accel_msg)

                # Erstellen und Publizieren der Nachricht für Gyroskop
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
