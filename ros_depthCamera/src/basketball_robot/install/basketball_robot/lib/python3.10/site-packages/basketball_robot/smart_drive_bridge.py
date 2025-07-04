#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Bool, String
import serial
import threading
import time
import math
import numpy as np
import os

class PIDController:
    def __init__(self, kp=0.5, ki=0.0, kd=0.05, sample_time=0.05):
        self.kp = kp          # Proportional gain
        self.ki = ki          # Integral gain
        self.kd = kd          # Derivative gain
        self.sample_time = sample_time  # Sample time in seconds
        
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
    def compute(self, error):
        current_time = time.time()
        delta_time = current_time - self.last_time
        
        # If adequate time has passed for a new calculation
        if delta_time >= self.sample_time:
            # Derivative term calculation
            derivative = (error - self.previous_error) / delta_time
            
            # Integral term calculation with anti-windup
            self.integral += error * delta_time
            self.integral = max(min(self.integral, 100), -100)  # Anti-windup limit
            
            # Calculate output
            output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
            
            # Store state for next iteration
            self.previous_error = error
            self.last_time = current_time
            
            return output
        else:
            # Not enough time has passed, return previous calculation
            return self.kp * error  # simplified response

class SmartDriveBridge(Node):
    def __init__(self):
        super().__init__('smart_drive_bridge')
        
        # Initialize running flag early to fix thread issue
        self.running = True
        
        # Parameters
        self.declare_parameter('dev', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('simulation_mode', False)
        self.declare_parameter('pid_distance_p', 0.5)
        self.declare_parameter('pid_distance_i', 0.01)
        self.declare_parameter('pid_distance_d', 0.1)
        self.declare_parameter('pid_alignment_p', 0.4)
        self.declare_parameter('pid_alignment_i', 0.0)
        self.declare_parameter('pid_alignment_d', 0.05)
        self.declare_parameter('target_distance', 2.5)  # Meters
        self.declare_parameter('camera_angle', 55.0)  # Degrees
        
        # Get parameters
        port = self.get_parameter('dev').value
        baud = self.get_parameter('baud').value
        self.simulation_mode = self.get_parameter('simulation_mode').value
        self.target_distance = self.get_parameter('target_distance').value
        self.camera_angle_degrees = self.get_parameter('camera_angle').value
        
        # PID parameters
        kp_distance = self.get_parameter('pid_distance_p').value
        ki_distance = self.get_parameter('pid_distance_i').value
        kd_distance = self.get_parameter('pid_distance_d').value
        kp_alignment = self.get_parameter('pid_alignment_p').value
        ki_alignment = self.get_parameter('pid_alignment_i').value
        kd_alignment = self.get_parameter('pid_alignment_d').value
        
        # Create log directory for Arduino communication
        self.log_dir = os.path.expanduser("~/arduino_logs")
        os.makedirs(self.log_dir, exist_ok=True)
        self.log_file = os.path.join(self.log_dir, f"arduino_log_{time.strftime('%Y%m%d_%H%M%S')}.txt")
        self.get_logger().info(f"Logging Arduino communication to {self.log_file}")
        
        # Open serial connection if not in simulation mode
        if not self.simulation_mode:
            try:
                self.ser = serial.Serial(port, baud, timeout=0.1)
                self.get_logger().info(f'Opened serial on {port} @ {baud}bps')
                
                # Flush any previous data
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to open serial port {port}: {e}')
                if not self.simulation_mode:
                    raise
                else:
                    self.get_logger().warn("Continuing in simulation mode despite serial error")
        else:
            self.get_logger().info("Running in simulation mode (no serial connection)")
        
        # Subscriptions
        self.create_subscription(Joy, '/joy', self.cb_joy, 10)
        self.create_subscription(Float32, '/basketball/distance', self.cb_distance, 10)
        self.create_subscription(Float32, '/basketball/x_offset', self.cb_x_offset, 10)
        self.create_subscription(Bool, '/basketball/detected', self.cb_detection_status, 10)
        
        # Publishers
        self.arduino_log_pub = self.create_publisher(String, '/arduino/log', 10)
        self.arduino_send_pub = self.create_publisher(String, '/arduino/sent_data', 10)
        self.arduino_recv_pub = self.create_publisher(String, '/arduino/received_data', 10)
        
        # State variables
        self.auto_mode = False
        self.current_distance = 0.0
        self.target_x_offset = 0.0
        self.target_detected = False
        
        # Initialize PID controllers
        self.distance_pid = PIDController(kp=kp_distance, ki=ki_distance, kd=kd_distance)
        self.alignment_pid = PIDController(kp=kp_alignment, ki=ki_alignment, kd=kd_alignment)
        
        # Latest joystick message storage
        self.latest_joy = None
        
        # Serial read thread
        if not self.simulation_mode:
            self.serial_thread = threading.Thread(target=self.read_from_serial)
            self.serial_thread.daemon = True
            self.serial_thread.start()
        
        # Auto control thread
        self.auto_thread = threading.Thread(target=self.auto_control_loop)
        self.auto_thread.daemon = True
        self.auto_thread.start()
        
        # Publish status info
        self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Smart Drive Bridge initialized')
    
    def log_arduino_message(self, direction, message):
        """Log Arduino message to file and publish to topic"""
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        log_entry = f"{timestamp} {direction}: {message}"
        
        # Log to console
        self.get_logger().info(log_entry)
        
        # Log to file
        with open(self.log_file, 'a') as f:
            f.write(log_entry + '\n')
        
        # Publish to topic
        log_msg = String()
        log_msg.data = log_entry
        self.arduino_log_pub.publish(log_msg)
    
    def read_from_serial(self):
        """Read data from Arduino serial port"""
        while self.running and not self.simulation_mode:
            try:
                if hasattr(self, 'ser') and self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line:
                        # Log received data
                        self.log_arduino_message("RECV", line)
                        
                        # Publish received data
                        recv_msg = String()
                        recv_msg.data = line
                        self.arduino_recv_pub.publish(recv_msg)
            except Exception as e:
                self.get_logger().error(f'Error reading from serial: {e}')
            
            time.sleep(0.01)  # Small delay
    
    def cb_distance(self, msg):
        """Receive distance from basketball detector"""
        self.current_distance = msg.data
    
    def cb_x_offset(self, msg):
        """Receive horizontal offset from basketball detector"""
        self.target_x_offset = msg.data
    
    def cb_detection_status(self, msg):
        """Receive detection status from basketball detector"""
        self.target_detected = msg.data
    
    def publish_status(self):
        """Publish status information"""
        if self.auto_mode:
            status = (f'AUTO MODE | Distance: {self.current_distance:.2f}m | '
                     f'Target: {self.target_distance:.2f}m | '
                     f'X-Offset: {self.target_x_offset:.2f} | '
                     f'Detected: {self.target_detected}')
        else:
            status = 'MANUAL MODE'
            
        self.get_logger().info(status)
        
        # Also log to Arduino log
        self.log_arduino_message("STATUS", status)
    
    def cb_joy(self, msg: Joy):
        """Process joystick messages"""
        self.latest_joy = msg
        
        # Auto mode toggle on button 7 press (one-time toggle)
        if len(msg.buttons) > 7:
            if msg.buttons[7] == 1 and not hasattr(self, 'last_button7') or (hasattr(self, 'last_button7') and self.last_button7 == 0 and msg.buttons[7] == 1):
                self.auto_mode = not self.auto_mode
                self.get_logger().info(f'Auto mode {"enabled" if self.auto_mode else "disabled"}')
                self.log_arduino_message("MODE", f'Auto mode {"enabled" if self.auto_mode else "disabled"}')
            self.last_button7 = msg.buttons[7]
        
        # In manual mode, directly send joystick data to Arduino
        if not self.auto_mode:
            self.send_control_data(msg.axes, msg.buttons)
    
    def send_control_data(self, axes, buttons):
        """Send control data to Arduino or simulation"""
        # Format axes with 2 decimal places for consistent length
        axes_str = ','.join(f'{a:.2f}' for a in axes)
        buttons_str = ','.join(str(b) for b in buttons)
        
        # Format exactly as Arduino expects: "axis_values;button_values"
        line = f'{axes_str};{buttons_str}\n'
        
        # Log the data being sent
        send_msg = String()
        send_msg.data = line.strip()
        self.arduino_send_pub.publish(send_msg)
        
        # Log the message
        mode = "AUTO" if self.auto_mode else "MANUAL"
        self.log_arduino_message(f"SEND ({mode})", line.strip())
        
        # Send to Arduino
        if not self.simulation_mode and hasattr(self, 'ser'):
            try:
                self.ser.write(line.encode())
            except Exception as e:
                self.get_logger().error(f"Serial write error: {e}")
    
    def auto_control_loop(self):
        """Thread for auto control mode"""
        while self.running:
            if self.auto_mode and self.target_detected:
                try:
                    # Calculate corrected distance based on camera angle
                    angle_rad = math.radians(self.camera_angle_degrees)
                    corrected_distance = self.current_distance * math.cos(angle_rad)
                    
                    # Calculate distance error (difference from target)
                    distance_error = corrected_distance - self.target_distance
                    
                    # Calculate alignment error (horizontal offset from center)
                    alignment_error = self.target_x_offset
                    
                    # Use PID to determine speeds
                    forward_speed = self.distance_pid.compute(distance_error)
                    forward_speed = max(min(forward_speed, 1.0), -1.0)  # Clamp to [-1.0, 1.0]
                    
                    lateral_speed = self.alignment_pid.compute(alignment_error)
                    lateral_speed = max(min(lateral_speed, 1.0), -1.0)  # Clamp to [-1.0, 1.0]
                    
                    # Create synthetic joystick message for auto movement
                    if self.latest_joy:
                        axes = list(self.latest_joy.axes)
                        buttons = list(self.latest_joy.buttons)
                        
                        # Override X and Y axes with PID outputs
                        # Y-axis for forward/backward
                        axes[1] = -forward_speed  # Negative because forward might be negative axis value
                        
                        # X-axis for left/right
                        axes[0] = -lateral_speed  # Negate as positive X might be right and target coordinates
                        
                        # Send to Arduino
                        self.send_control_data(axes, buttons)
                except Exception as e:
                    self.get_logger().error(f'Error in auto control loop: {e}')
            elif self.auto_mode and not self.target_detected:
                # Stop the robot if no target is detected but in auto mode
                if self.latest_joy:
                    # Create a zeroed-out axis list but keep buttons as-is
                    axes = [0.0] * len(self.latest_joy.axes)
                    buttons = list(self.latest_joy.buttons)
                    self.send_control_data(axes, buttons)
            
            # Sleep to avoid hammering the CPU
            time.sleep(0.05)  # 20 Hz update rate

def main(args=None):
    rclpy.init(args=args)
    node = SmartDriveBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        if not node.simulation_mode and hasattr(node, 'ser'):
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()