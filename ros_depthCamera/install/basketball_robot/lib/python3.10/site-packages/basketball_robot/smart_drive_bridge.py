#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
import serial
import threading
import time

class JoyForwarder(Node):
    def __init__(self):
        super().__init__('joy_forwarder')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('auto_align_button', 7)  # Button 8 (index 7) for auto alignment
        self.declare_parameter('auto_align_button_Emergency', 8)   
        self.declare_parameter('override_button', 9)     # Button 10 (index 9) for override/stop
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.auto_align_button = self.get_parameter('auto_align_button').value
        self.override_button = self.get_parameter('override_button').value
        
        # Initialize state
        self.auto_mode = False
        self.last_auto_button_state = False
        self.override = False
        self.last_override_button_state = False
        self.pid_x = 0.0
        self.pid_y = 0.0
        self.last_debug_print = 0
        self.joy_data = None
        self.last_joy_time = time.time()
        
        # Setup serial connection
        self.serial_lock = threading.Lock()
        try:
            self.serial = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            # Clear any garbage in the buffer
            time.sleep(0.2)
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
            self.serial_connected = True
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.serial_connected = False
        
        # Create subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.value_x_sub = self.create_subscription(Float32, '/value_x', self.value_x_callback, 10)
        self.value_y_sub = self.create_subscription(Float32, '/value_y', self.value_y_callback, 10)
        
        # Timer for sending data
        self.timer = self.create_timer(0.05, self.send_control_data)  # 20Hz
        
        self.get_logger().info("Joy forwarder initialized")
        self.get_logger().info(f"Button {self.auto_align_button} toggles auto alignment mode")
        self.get_logger().info(f"Button {self.override_button} is safety override (stop)")
    
    def joy_callback(self, msg):
        """Store joystick data and check for mode toggles"""
        # Store the joystick data
        self.joy_data = msg
        self.last_joy_time = time.time()
        
        # Check for auto alignment toggle (Button 7)
        if len(msg.buttons) > self.auto_align_button :
            auto_button_state = bool(msg.buttons[self.auto_align_button])
            
            # Toggle auto mode on button press (not release)
            if auto_button_state and not self.last_auto_button_state:
                self.auto_mode = not self.auto_mode
                mode_msg = "AUTO ALIGNMENT" if self.auto_mode else "MANUAL"
                self.get_logger().info(f"Switched to {mode_msg} control mode")
            
            self.last_auto_button_state = auto_button_state
            
        # Check for override toggle (Button 9)
        if len(msg.buttons) > self.override_button:
            override_button_state = bool(msg.buttons[self.override_button])
            
            # Update override state
            if override_button_state != self.last_override_button_state:
                self.override = override_button_state
                if self.override:
                    self.get_logger().info("OVERRIDE ACTIVE - Motors stopped")
                else:
                    self.get_logger().info("Override released")
            
            self.last_override_button_state = override_button_state
        
        # Print debug info periodically
        current_time = time.time()
        if current_time - self.last_debug_print > 1.0:
            self.last_debug_print = current_time
            self.get_logger().info("=== Joystick Data ===")
            self.get_logger().info(f"Axes: {[round(a, 2) for a in msg.axes]}")
            self.get_logger().info(f"Buttons: {msg.buttons[:15]}")
            self.get_logger().info(f"Mode: {'AUTO ALIGN' if self.auto_mode else 'MANUAL'}, Override: {self.override}")
    
    def value_x_callback(self, msg):
        """Store X PID value"""
        self.pid_x = msg.data
    
    def value_y_callback(self, msg):
        """Store Y PID value"""
        self.pid_y = msg.data
    
    def send_control_data(self):
        """Send appropriate control data based on mode"""
        if not self.serial_connected or self.joy_data is None:
            return
            
        try:
            # Ensure we have fresh joystick data (within last 0.5 seconds)
            if time.time() - self.last_joy_time > 0.5:
                return
                
            with self.serial_lock:
                if self.auto_mode and not self.override:
                    # In auto mode and not overridden, use PID values for axis[0] and axis[1]
                    fake_axes = list(self.joy_data.axes)  # Copy existing axes
                    fake_buttons = list(self.joy_data.buttons)  # Copy existing buttons
                    print(fake_axes, fake_buttons)
                    # Override the first two axes with PID values
                    if len(fake_axes) >= 2:
                        fake_axes[2] = self.pid_x
                        fake_axes[3] = self.pid_y
                    
                    # Format data for Arduino
                    axes_str = ','.join([f"{a}" for a in fake_axes])
                    buttons_str = ','.join([f"{b}" for b in fake_buttons])
                    line = f"{axes_str};{buttons_str}\n"
                    
                    # Send to Arduino
                    self.serial.write(line.encode())
                    
                    # Log periodically
                    if time.time() % 1 < 0.05:
                        self.get_logger().info(f"AUTO ALIGN - Sent: X={self.pid_x:.3f}, Y={self.pid_y:.3f}")
                else:

                    axes = ','.join(f'{a:.2f}' for a in self.joy_data.axes)
                    buttons = ','.join(str(b) for b in self.joy_data.buttons)
                    line = f'{axes};{buttons}\n'
                    self.serial.write(line.encode())
                    
        except Exception as e:
            self.get_logger().error(f"Error sending data: {e}")
            self.serial_connected = False
    
    def destroy_node(self):
        """Clean up when node is shut down"""
        if hasattr(self, 'serial') and self.serial_connected:
            try:
                # Send zeros before closing to ensure motors stop
                if self.joy_data and len(self.joy_data.axes) > 0:
                    axes_count = len(self.joy_data.axes)
                    buttons_count = len(self.joy_data.buttons)
                    
                    # Set override button to 1 to ensure stop
                    zero_axes = ','.join(['0.0'] * axes_count)
                    zero_buttons = ['0'] * buttons_count
                    if self.override_button < buttons_count:
                        zero_buttons[self.override_button] = '1'  # Set override button
                    
                    zero_buttons_str = ','.join(zero_buttons)
                    stop_data = f"{zero_axes};{zero_buttons_str}\n"
                    
                    self.serial.write(stop_data.encode())
                    time.sleep(0.1)  # Give time for command to be sent
                
                self.serial.close()
                self.get_logger().info("Serial connection closed")
            except Exception as e:
                self.get_logger().error(f"Error closing serial: {e}")
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = JoyForwarder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Interrupted by keyboard")
    except Exception as e:
        if node:
            node.get_logger().error(f"Error: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()