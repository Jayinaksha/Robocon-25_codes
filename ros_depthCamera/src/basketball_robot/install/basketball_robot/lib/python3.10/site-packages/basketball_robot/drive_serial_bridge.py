#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import threading
import numpy as np
import cv2
import pyrealsense2 as rs
import time
import math
import os

# Try to import YOLO with error handling
try:
    from ultralytics import YOLO
except ImportError:
    print("Installing ultralytics...")
    os.system("pip install ultralytics")
    from ultralytics import YOLO

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

class DriveSerialBridge(Node):
    def __init__(self):
        super().__init__('drive_serial_bridge')
        
        # Serial parameters
        self.declare_parameter('dev', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('pid_distance_p', 0.5)
        self.declare_parameter('pid_distance_i', 0.01)
        self.declare_parameter('pid_distance_d', 0.1)
        self.declare_parameter('pid_alignment_p', 0.4)
        self.declare_parameter('pid_alignment_i', 0.0)
        self.declare_parameter('pid_alignment_d', 0.05)
        
        port = self.get_parameter('dev').value
        baud = self.get_parameter('baud').value
        kp_distance = self.get_parameter('pid_distance_p').value
        ki_distance = self.get_parameter('pid_distance_i').value
        kd_distance = self.get_parameter('pid_distance_d').value
        kp_alignment = self.get_parameter('pid_alignment_p').value
        ki_alignment = self.get_parameter('pid_alignment_i').value
        kd_alignment = self.get_parameter('pid_alignment_d').value
        
        # Target parameters for basketball shooting
        self.declare_parameter('target_distance', 2.0)  # Meters
        self.declare_parameter('camera_angle', 55.0)  # Degrees
        
        self.target_distance = self.get_parameter('target_distance').value
        self.camera_angle_degrees = self.get_parameter('camera_angle').value
        
        # Open serial connection
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Opened serial on {port} @ {baud}bps')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            raise
        
        # Joy subscription
        self.create_subscription(Joy, '/joy', self.cb_joy, 10)
        
        # State variables
        self.auto_mode = False
        self.current_distance = 0.0
        self.vision_initialized = False
        self.target_x_offset = 0.0  # Horizontal offset from center
        
        # Initialize PID controllers
        self.distance_pid = PIDController(kp=kp_distance, ki=ki_distance, kd=kd_distance)
        self.alignment_pid = PIDController(kp=kp_alignment, ki=ki_alignment, kd=kd_alignment)
        
        # Latest joystick message storage
        self.latest_joy = None
        
        # Threading control
        self.running = True
        self.auto_thread = threading.Thread(target=self.auto_control_loop)
        self.auto_thread.daemon = True
        self.auto_thread.start()
        
        # Initialize RealSense and YOLO in a separate thread to not block the node
        self.vision_thread = threading.Thread(target=self.init_vision)
        self.vision_thread.daemon = True
        self.vision_thread.start()
        
        # Publish status info
        self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Drive Serial Bridge initialized')
        
    def init_vision(self):
        """Initialize RealSense camera and YOLO model"""
        try:
            self.get_logger().info('Initializing RealSense pipeline...')
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.pipeline.start(self.config)
            
            # Configure depth stream post-processing
            self.align = rs.align(rs.stream.color)
            
            self.get_logger().info('Loading YOLO model...')
            # First check for a basketball hoop model
            model_path = "basketball_hoop.pt"
            if not os.path.exists(model_path):
                self.get_logger().info('Basketball hoop model not found, using default model')
                model_path = "yolov8n.pt"  # Use default model
                
            self.model = YOLO(model_path)
            self.vision_initialized = True
            self.get_logger().info('Vision system initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize vision system: {e}')
            self.vision_initialized = False
    
    def publish_status(self):
        """Publish status information"""
        if self.auto_mode:
            self.get_logger().info(f'Status: AUTO mode | '
                                   f'Distance: {self.current_distance:.2f}m | '
                                   f'Target: {self.target_distance:.2f}m | '
                                   f'X-Offset: {self.target_x_offset:.2f}')
        else:
            self.get_logger().info('Status: MANUAL mode')
    
    def cb_joy(self, msg: Joy):
        """Process joystick messages"""
        self.latest_joy = msg
        
        # Auto mode toggle on button 7 press (one-time toggle)
        if len(msg.buttons) > 7:
            if msg.buttons[7] == 1 and not hasattr(self, 'last_button7') or (hasattr(self, 'last_button7') and self.last_button7 == 0 and msg.buttons[7] == 1):
                self.auto_mode = not self.auto_mode
                self.get_logger().info(f'Auto mode {"enabled" if self.auto_mode else "disabled"}')
            self.last_button7 = msg.buttons[7]
        
        # In manual mode, directly send joystick data to Arduino
        if not self.auto_mode:
            axes = ','.join(f'{a:.2f}' for a in msg.axes)
            buttons = ','.join(str(b) for b in msg.buttons)
            line = f'{axes};{buttons}\n'
            self.ser.write(line.encode())
    
    def auto_control_loop(self):
        """Thread for auto control mode"""
        while self.running:
            if self.auto_mode and self.vision_initialized:
                try:
                    # Get current distance and horizontal offset from the vision system
                    detection_result = self.detect_target()
                    self.current_distance = detection_result['distance']
                    self.target_x_offset = detection_result['x_offset']
                    
                    if self.current_distance > 0:  # Valid detection
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
                        
                        self.get_logger().debug(
                            f'Auto: Dist={corrected_distance:.2f}m, Target={self.target_distance:.2f}m, '
                            f'Dist_Err={distance_error:.2f}m, X_Err={alignment_error:.2f}, '
                            f'Fwd_Speed={forward_speed:.2f}, Lat_Speed={lateral_speed:.2f}'
                        )
                        
                        # Create synthetic joystick message for auto movement
                        if self.latest_joy:
                            axes = list(self.latest_joy.axes)
                            buttons = list(self.latest_joy.buttons)
                            
                            # Override X and Y axes with PID outputs
                            # Y-axis for forward/backward
                            axes[1] = -forward_speed  # Negative because forward might be negative axis value
                            
                            # X-axis for left/right
                            axes[0] = -lateral_speed  # Negate as positive X might be right and target coordinates have 
                                                     # positive X on the right side of the image
                            
                            # Send to Arduino
                            axes_str = ','.join(f'{a:.2f}' for a in axes)
                            buttons_str = ','.join(str(b) for b in buttons)
                            line = f'{axes_str};{buttons_str}\n'
                            self.ser.write(line.encode())
                    else:
                        self.get_logger().warn('No valid target detected in auto mode')
                        # Stop the robot if no target is detected
                        if self.latest_joy:
                            axes = [0.0] * len(self.latest_joy.axes)
                            buttons = list(self.latest_joy.buttons)
                            axes_str = ','.join(f'{a:.2f}' for a in axes)
                            buttons_str = ','.join(str(b) for b in buttons)
                            line = f'{axes_str};{buttons_str}\n'
                            self.ser.write(line.encode())
                            
                except Exception as e:
                    self.get_logger().error(f'Error in auto control loop: {e}')
            
            # Sleep to avoid hammering the CPU
            time.sleep(0.05)  # 20 Hz update rate
    
    def detect_target(self):
        """Get distance and horizontal offset to target using RealSense and YOLO"""
        try:
            # Wait for frames
            frames = self.pipeline.wait_for_frames()
            
            # Align depth frame to color frame
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                return {'distance': -1, 'x_offset': 0}  # Invalid frames
                
            # Convert to numpy array
            color_image = np.asanyarray(color_frame.get_data())
            
            # Get image dimensions
            height, width = color_image.shape[:2]
            image_center_x = width / 2
            
            # Run YOLO detection
            results = self.model(color_image)
            result = results[0]
            
            # Find basketball hoop detection
            best_conf = 0
            target_distance = -1
            target_x_offset = 0
            
            for box in result.boxes:
                # Get box coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # Get confidence
                conf = float(box.conf[0])
                
                # Get class ID and name
                class_id = int(box.cls[0])
                class_name = result.names[class_id]
                
                # Check if this is a basketball hoop or relevant target
                if conf > 0.5 and (class_name == "basketball hoop" or 
                                   class_name == "sports ball" or
                                   class_name == "person"):  # Adjust class names as needed
                    
                    if conf > best_conf:
                        best_conf = conf
                        
                        # Calculate center point
                        center_x = (x1 + x2) // 2
                        center_y = (y1 + y2) // 2
                        
                        # Calculate horizontal offset from center (-1 to 1)
                        # Negative means target is to the left, positive means target is to the right
                        target_x_offset = (center_x - image_center_x) / image_center_x
                        
                        # Get distance at center point
                        target_distance = depth_frame.get_distance(center_x, center_y)
                        
                        # Draw visualization 
                        cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.line(color_image, (int(image_center_x), center_y), (center_x, center_y), (0, 0, 255), 2)
                        label = f"{class_name} {conf:.2f}, {target_distance:.2f}m, x_off:{target_x_offset:.2f}"
                        cv2.putText(color_image, label, (x1, y1-10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
            # Show the detection image
            cv2.imshow("Basketball Detection", color_image)
            cv2.waitKey(1)
                        
            return {'distance': target_distance, 'x_offset': target_x_offset}
            
        except Exception as e:
            self.get_logger().error(f'Error detecting target: {e}')
            return {'distance': -1, 'x_offset': 0}

def main(args=None):
    rclpy.init(args=args)
    node = DriveSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        if hasattr(node, 'pipeline'):
            node.pipeline.stop()
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()