# ESP32 Joystick Data Receiver for Robot Control

## Overview

This project features an ESP32-based joystick data receiver integrated with a ROS 2 `joy_serial_bridge` package. The system enables control of a 4-wheeled drive robot equipped with a flywheel shooting mechanism, an adjustable angle motor, and a servo. The joystick axes and button inputs are processed and mapped to control the robot's motors and mechanisms.

## Features

- **Joystick Input Parsing**: Receives joystick data in the format `"axes;buttons"` via ROS and parses it.
- **4-Wheeled Drive Control**: Maps joystick axes to control forward, backward, left, and right movement.
- **Flywheel Shooting Mechanism**: Activates flywheel motors via specific joystick buttons.
- **Servo Control**: Adjusts servo position using joystick buttons.
- **Angle Adjustment**: Uses a dedicated motor to change the flywheel's shooting angle based on joystick input.
- **Dynamic Feedback**: Debugging information is printed to the serial monitor, including axis values, button states, and robot actions.

## System Components

### Hardware
1. **ESP32 Microcontroller**
2. **4 DC Motors for Driving**
3. **2 Motors for Flywheel Shooting Mechanism**
4. **1 Motor for Angle Adjustment**
5. **1 Servo for fine control**
6. **Joystick Controller**

### ROS 2 Package
- **joy_serial_bridge**: Bridges joystick input `/joy` from ROS to serial data sent to the ESP32.

## Installation

### Prerequisites
1. **ROS 2** (Humble or jazzy or later)
2. Python dependencies for ROS packages:
   ```bash
   sudo apt install python3-serial
   ```

3. **Hardware Setup**:
   - Connect the ESP32 to a computer via USB.
   - Wire the motors, servo, and flywheel mechanism according to your robot's design.

### Setting Up ROS Workspace
1. Create a ROS workspace:
   ```bash
   mkdir -p ~/joy_serial_ws/src
   cd ~/joy_serial_ws/src
   ros2 pkg create --build-type ament_python joy_serial_bridge
   ```
2. Add `joy_serial_bridge` code to the newly created package and build:
   ```bash
   cd ~/joy_serial_ws
   colcon build --packages-select joy_serial_bridge
   source install/setup.bash
   ```

3. Verify package visibility:
   ```bash
   ros2 pkg list | grep joy_serial_bridge
   ```

### Upload ESP32 Code
1. Open the Arduino IDE and load the `joy_serial_receiver.ino` file provided in this repository.
2. Configure the serial monitor baud rate to `115200`.
3. Upload the code to the ESP32.

## Usage

### Starting ROS 2 Nodes
1. Start the joystick node:
   ```bash
   ros2 run joy joy_node --ros-args -p deadzone:=0.0
   ```
2. Start the `joy_serial_bridge`:
   ```bash
   ros2 run joy_serial_bridge joy_to_serial --ros-args -p dev:=/dev/ttyUSB0 -p baud:=115200
   ```

### Robot Control
- **Driving**: Use the left joystick to control forward, backward, left, and right movement.
- **Flywheel Shooting**: Activate the flywheel motors using the assigned button.
- **Angle Adjustment**: Adjust the shooting angle using the corresponding joystick axis.
- **Servo Control**: Fine-tune shooting angle using the servo control buttons.

### Debugging
Monitor joystick inputs and robot actions via the Arduino Serial Monitor or an alternative terminal:
```bash
sudo apt install minicom
```
```bash
minicom -D /dev/ttyUSB0 -b 115200
```
You should see parsed joystick data and corresponding robot actions.

## Notes
1. Adjust the axis/button mappings in the ESP32 code (`joy_serial_receiver.ino`) to suit your robot's control requirements.
2. Ensure your motors and servo are configured correctly for the ESP32 pins.
