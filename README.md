# ROS2-Node for IMU "MPU6050"

This repository contains a ROS2 package that reads data from an MPU6050 sensor connected via a serial port and publishes the acceleration and gyroscope data as ROS messages. The main node is implemented in the `leap_imu.py` file with the `MPU6050Node` class.

## Prerequisites

- ROS2 (tested on ROS2 Jazzy)
- Python 3
- Serial port access (sensor connected under `/dev/ttyACM0`)

## Build Instructions

1. Open a terminal in the root of the repository.
2. Run the following command to build the workspace:

   ```sh
   colcon build
   ```
3. After building, source the setup file:
    
   ```sh
   source install/setup.bash
   ```

## Running the Node

After sourcing the environment, start the ROS2 node by running:

```sh
ros2 run leap_imu leap_imu
```

This command will launch the node, which will then read the data from the serial port and publish the IMU data.

Adjust the serial port path and baud rate in `leap_imu.py` if needed.

## Listening to Topics

You can listen to the published topics by running:

```sh
ros2 topic echo /mpu6050/acceleration
```

```sh
ros2 topic echo /mpu6050/gyroscope
```

## Uploading the Arduino Code

To upload the Arduino sketch:
- Install the Arduino IDE.
- Open the file "arduino_code.ino" with the IDE.
- Compile and upload the sketch to your Arduino.
- Import any missing libraries if prompted.