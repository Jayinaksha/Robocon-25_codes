#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import time

class JoyToROSSafe(Node):
    def __init__(self):
        super().__init__('joy_to_ros_safe')
        
        # Subscribe to PS4 controller
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Publisher to send PS4 data to smart_drive_bridge
        self.joy_data_pub = self.create_publisher(String, '/joy_serial_data', 10)
        
        # Store previous button states
        self.prev_buttons = None
        self.last_button_time = {}
        
        self.get_logger().info("✅ Joy to ROS Bridge (Safe Mode) ready")
        self.get_logger().info("🎮 Only single button presses will be detected")
    
    def joy_callback(self, msg):
        """Handle PS4 controller input"""
        buttons = list(msg.buttons)
        axes = list(msg.axes)
        
        current_time = time.time()
        
        # Only process if exactly one button is pressed
        pressed_buttons = [i for i, b in enumerate(buttons) if b == 1]
        
        if len(pressed_buttons) == 1:  # Exactly one button pressed
            button_index = pressed_buttons[0]
            
            # Check if this is a new press (not held)
            if self.prev_buttons is None or self.prev_buttons[button_index] == 0:
                # Check if enough time has passed since last press of this button
                if button_index not in self.last_button_time or \
                   current_time - self.last_button_time[button_index] > 0.5:  # 500ms debounce
                    
                    self.handle_single_button_press(button_index)
                    self.last_button_time[button_index] = current_time
        
        # Update previous button states
        self.prev_buttons = buttons.copy()
        
        # Send controller data
        self.send_controller_data(buttons, axes)
    
    def handle_single_button_press(self, button_index):
        """Handle single button press"""
        self.get_logger().info(f"🎮 Single button press detected: {button_index}")
        
        if button_index == 6:  # Button 7
            self.get_logger().info("🤖 Button 7 (Autonomous Mode)")
            self.joy_data_pub.publish(String(data="BUTTON_7_PRESSED"))
            
        elif button_index == 7:  # Button 8
            self.get_logger().info("🎯 Button 8 (Shoot Permission)")
            self.joy_data_pub.publish(String(data="BUTTON_8_PRESSED"))
            
        elif button_index == 8:  # Button 9
            self.get_logger().info("🛑 Button 9 (Emergency Stop)")
            self.joy_data_pub.publish(String(data="BUTTON_9_PRESSED"))
    
    def send_controller_data(self, buttons, axes):
        """Send formatted controller data"""
        if any(buttons) or any(abs(axis) > 0.1 for axis in axes):
            controller_data = f"PS4:{','.join(str(int(b)) for b in buttons)}:{','.join(f'{a:.3f}' for a in axes)}"
            self.joy_data_pub.publish(String(data=controller_data))

def main(args=None):
    rclpy.init(args=args)
    joy_to_ros = JoyToROSSafe()
    
    try:
        rclpy.spin(joy_to_ros)
    except KeyboardInterrupt:
        joy_to_ros.get_logger().info("🛑 Shutting down...")
    finally:
        joy_to_ros.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
