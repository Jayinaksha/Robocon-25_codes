#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import time

class JoyToROS(Node):
    def __init__(self):
        super().__init__('joy_to_ros')
        
        # Subscribe to PS4 controller
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Publisher to send PS4 data to smart_drive_bridge
        self.joy_data_pub = self.create_publisher(String, '/joy_serial_data', 10)
        
        # Store previous button states to detect button presses
        self.prev_buttons = None
        
        self.get_logger().info("✅ Joy to ROS Bridge ready")
        self.get_logger().info("🎮 Converting PS4 controller data to ROS messages")
        self.get_logger().info("📡 Publishing to /joy_serial_data topic for smart_drive_bridge")
        self.get_logger().info("🔧 Handles YAML format from joy_node")
    
    def joy_callback(self, msg):
        """Handle PS4 controller input"""
        # Get current button states - convert to list
        buttons = list(msg.buttons)
        axes = list(msg.axes)
        
        # DEBUG: Print button states when any button is pressed
        if any(buttons):
            pressed_buttons = [i for i, b in enumerate(buttons) if b == 1]
            self.get_logger().info(f"🔍 DEBUG: Pressed buttons (indices): {pressed_buttons}")
        
        # Check for button presses (only trigger on button press, not hold)
        if self.prev_buttons is not None:
            for i, (current, previous) in enumerate(zip(buttons, self.prev_buttons)):
                if current == 1 and previous == 0:  # Button pressed (transition from 0 to 1)
                    self.handle_button_press(i, buttons, axes)
        
        # Update previous button states
        self.prev_buttons = buttons.copy()
        
        # Send controller data in YAML format (for Arduino compatibility)
        self.send_yaml_format(buttons, axes)
        
        # Also send PS4 string format (for backward compatibility)
        self.send_ps4_string_format(buttons, axes)
    
    def handle_button_press(self, button_index, buttons, axes):
        """Handle specific button presses"""
        self.get_logger().info(f"🎮 Button index {button_index} pressed")
        
        # Correct button index mapping
        if button_index == 7:  # Button 7 (0-based index 7) - Autonomous Mode
            self.get_logger().info("🤖 Button 7 pressed - Autonomous Mode Toggle")
            self.joy_data_pub.publish(String(data="BUTTON_7_PRESSED"))
            
        elif button_index == 8:  # Button 8 (0-based index 8) - Shoot Permission
            self.get_logger().info("🎯 Button 8 pressed - Shoot Permission")
            self.joy_data_pub.publish(String(data="BUTTON_8_PRESSED"))
            
        elif button_index == 9:  # Button 9 (0-based index 9) - Emergency Stop
            self.get_logger().info("🛑 Button 9 pressed - Emergency Stop")
            self.joy_data_pub.publish(String(data="BUTTON_9_PRESSED"))
            
        else:
            self.get_logger().debug(f"🎮 Button index {button_index} pressed (not mapped)")
    
    def send_yaml_format(self, buttons, axes):
        """Send data in YAML format that Arduino can parse"""
        # Ensure we have enough buttons and axes
        buttons = buttons + [0] * (20 - len(buttons))  # Pad to 20 buttons
        axes = axes + [0.0] * (8 - len(axes))  # Pad to 8 axes
        
        # Create YAML format string
        yaml_lines = []
        
        # Send axes header
        yaml_lines.append("axes:")
        for axis in axes:
            yaml_lines.append(f"- {axis:.6f}")
        
        # Send buttons header
        yaml_lines.append("buttons:")
        for button in buttons:
            yaml_lines.append(f"- {button}")
        
        # Send end marker
        yaml_lines.append("---")
        
        # Publish each line separately (as Arduino expects line-by-line)
        for line in yaml_lines:
            self.joy_data_pub.publish(String(data=line))
            time.sleep(0.001)  # Small delay between lines
        
        # Debug: Log data occasionally
        if any(buttons) or any(abs(axis) > 0.1 for axis in axes):
            self.get_logger().debug(f"📤 Sent YAML format data")
    
    def send_ps4_string_format(self, buttons, axes):
        """Send data in PS4 string format (for backward compatibility)"""
        # Ensure we have enough buttons and axes
        buttons = buttons + [0] * (20 - len(buttons))  # Pad to 20 buttons
        axes = axes + [0.0] * (8 - len(axes))  # Pad to 8 axes
        
        # Create PS4 string format
        button_str = ','.join(str(int(b)) for b in buttons)
        axis_str = ','.join(f"{a:.6f}" for a in axes)
        
        # Arduino expects: PS4:axes:buttons
        ps4_data = f"PS4:{axis_str}:{button_str}"
        
        # Publish PS4 format
        self.joy_data_pub.publish(String(data=ps4_data))
        
        # Debug: Log data occasionally
        if any(buttons) or any(abs(axis) > 0.1 for axis in axes):
            self.get_logger().debug(f"📤 Sent PS4 string format data")

def main(args=None):
    rclpy.init(args=args)
    joy_to_ros = JoyToROS()
    
    try:
        rclpy.spin(joy_to_ros)
    except KeyboardInterrupt:
        joy_to_ros.get_logger().info("🛑 Shutting down Joy to ROS Bridge...")
    finally:
        joy_to_ros.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()