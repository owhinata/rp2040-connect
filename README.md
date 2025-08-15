# Arduino Nano RP2040 Connect - ROS2 Integration

## Overview

This repository contains Arduino sketches and ROS2 integration examples for the Arduino Nano RP2040 Connect board. It demonstrates how to use the board's built-in sensors (particularly the IMU) and integrate with ROS2 using micro-ROS.

## Hardware Requirements

- **Arduino Nano RP2040 Connect**: A powerful microcontroller board featuring:
  - Raspberry Pi RP2040 dual-core Cortex-M0+ processor
  - Built-in LSM6DSOX 6-axis IMU (accelerometer + gyroscope)
  - Wi-Fi and Bluetooth connectivity via u-blox NINA-W102 module
  - Operating voltage: 3.3V
- **USB Cable**: For programming and serial communication
- **Host Computer**: Running Ubuntu with ROS2 Humble installed

## Prerequisites

- Arduino CLI installed on your system
- ROS2 Humble distribution
- User added to `dialout` group: `sudo usermod -a -G dialout $USER`

## Setup

### 1. Install Arduino Core for RP2040

```bash
arduino-cli core install arduino:mbed_nano
```

### 2. Configure USB Permissions (Linux)

If you encounter upload failures, configure udev rules to allow non-root access to the device:

```bash
# Create udev rules file
sudo vi /etc/udev/rules.d/99-arduino-nano-rp2040.rules

# Add the following lines:
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="005e", MODE="0666", GROUP="dialout"
SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0003", MODE="0666", GROUP="dialout"

# Reload and apply the rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Examples

### 1. LED Blink Test

A simple example to verify board functionality by blinking the built-in LED.

```bash
# Compile the sketch
arduino-cli compile --fqbn arduino:mbed_nano:nanorp2040connect Led

# Upload to the board
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:mbed_nano:nanorp2040connect Led
```

### 2. IMU Sensor Reading

Read data from the built-in LSM6DSOX 6-axis IMU sensor.

#### Install Required Library

```bash
arduino-cli lib install Arduino_LSM6DSOX
```

#### Build and Run

```bash
# Compile the IMU sketch
arduino-cli compile --fqbn arduino:mbed_nano:nanorp2040connect IMU

# Upload to the board
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:mbed_nano:nanorp2040connect IMU

# Monitor serial output to see IMU data
arduino-cli monitor -p /dev/ttyACM0
```

### 3. Micro-ROS Basic Example

Integrate the Arduino with ROS2 using micro-ROS to publish a simple counter.

#### Install Required Libraries

```bash
# Install micro-ROS Arduino library (version compatible with ROS2 Humble)
arduino-cli lib install micro_ros_arduino@2.0.7-humble

# Install WiFi library (required dependency)
arduino-cli lib install WiFiNINA
```

#### Build and Upload Arduino Sketch

```bash
# Compile the micro-ROS sample
arduino-cli compile --fqbn arduino:mbed_nano:nanorp2040connect MicroRosSample

# Upload to the board
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:mbed_nano:nanorp2040connect MicroRosSample
```

#### Setup and Run Micro-ROS Agent

The micro-ROS agent acts as a bridge between the Arduino and ROS2 network.

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Initialize git submodules (if any)
git submodule update --init --recursive

# Build the ROS2 workspace
cd ros2_ws
colcon build
source install/setup.bash

# Create micro-ROS agent workspace
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/setup.bash

# Launch the micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial -D /dev/ttyACM0 -b 115200 -v6
```

#### Verify Communication

With the device connected and agent running:

```bash
# List available topics
ros2 topic list

# Echo the counter topic published by Arduino
ros2 topic echo /pico_counter
```

### 4. Micro-ROS IMU Publisher

Advanced example that publishes IMU sensor data to ROS2 topics with enable/disable control.

#### Build and Upload

```bash
# Compile the micro-ROS IMU sketch
arduino-cli compile --fqbn arduino:mbed_nano:nanorp2040connect MicroRosIMU

# Upload to the board
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:mbed_nano:nanorp2040connect MicroRosIMU
```

#### Run the Agent

```bash
# Launch micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial -D /dev/ttyACM0 -b 115200 -v
```

#### Control and Monitor IMU Data

With the device connected and agent running:

```bash
# List available topics
ros2 topic list

# Enable IMU publishing and echo the data
ros2 topic pub -1 /imu_enable std_msgs/Bool "data: true" && ros2 topic echo /imu/data_raw

# Disable IMU publishing
ros2 topic pub -1 /imu_enable std_msgs/Bool "data: false"
```

## Project Structure

```
.
├── Led/                 # LED blink example
├── IMU/                 # Standalone IMU reading example
├── MicroRosSample/      # Basic micro-ROS publisher example
├── MicroRosIMU/         # IMU data publisher with ROS2 integration
└── ros2_ws/             # ROS2 workspace for micro-ROS agent
```

## Troubleshooting

1. **Upload failures**: Ensure USB permissions are correctly set (see Setup section)
2. **Serial port not found**: Check device with `ls /dev/ttyACM*` and adjust port accordingly
3. **Micro-ROS connection issues**: Verify baud rate (115200) and that ROS2 environment is sourced
4. **IMU data not publishing**: Ensure the `/imu_enable` topic is set to `true`

## Additional Resources

- [Arduino Nano RP2040 Connect Documentation](https://docs.arduino.cc/hardware/nano-rp2040-connect)
- [micro-ROS for Arduino](https://github.com/micro-ROS/micro_ros_arduino)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)

