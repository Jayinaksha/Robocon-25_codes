#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import math
import time
import os
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32

# Try importing RealSense and YOLO
try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
    print("✅ RealSense SDK available")
except ImportError:
    REALSENSE_AVAILABLE = False
    print("❌ RealSense SDK not available")

try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
    print("✅ cv_bridge available")
except ImportError:
    CV_BRIDGE_AVAILABLE = False
    print("❌ cv_bridge not available")

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
    print("✅ YOLO available")
except ImportError:
    YOLO_AVAILABLE = False
    print("❌ YOLO not available")

# PID Controller Class
class PIDController:
    def __init__(self, kp, ki, kd, output_min=-1.0, output_max=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        
        # Internal state
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
    
    def compute(self, error):
        # Calculate time delta
        current_time = time.time()
        dt = current_time - self.last_time
        
        # Avoid division by zero or negative time (can happen when time wraps around)
        if dt <= 0:
            dt = 0.01
            
        # Calculate derivative
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        
        # Update integral with anti-windup
        self.integral += error * dt
        
        # Calculate PID output
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        # Clamp output
        output = max(self.output_min, min(self.output_max, output))
        
        # Save state for next iteration
        self.previous_error = error
        self.last_time = current_time
        
        return output
    
    def reset(self):
        """Reset PID internal state"""
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

class RealsenseBasketDetector(Node):
    def __init__(self):
        super().__init__('realsense_basket_detector')
        
        # Parameters
        self.declare_parameter('model_path', '/workspace/Robocon25_codes/ros_depthCamera/src/basketball_robot/basketball_robot/last.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('shooting_angle', 55.0)  # degrees
        self.declare_parameter('target_distance', 5.726)  # meters
        
        # PID Parameters
        self.declare_parameter('x_kp', .5)
        self.declare_parameter('x_ki', 0.0)
        self.declare_parameter('x_kd', 0.0)
        self.declare_parameter('y_kp', 0.5)
        self.declare_parameter('y_ki', 0.0)
        self.declare_parameter('y_kd', 0.0)
        
        # Get basic parameters
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.shooting_angle = self.get_parameter('shooting_angle').value
        self.target_distance = self.get_parameter('target_distance').value
        
        # Get PID parameters
        x_kp = self.get_parameter('x_kp').value
        x_ki = self.get_parameter('x_ki').value
        x_kd = self.get_parameter('x_kd').value
        y_kp = self.get_parameter('y_kp').value
        y_ki = self.get_parameter('y_ki').value
        y_kd = self.get_parameter('y_kd').value
        
        # Initialize PID controllers
        self.x_pid = PIDController(x_kp, x_ki, x_kd, -0.5, 0.5)  # Horizontal control
        self.y_pid = PIDController(y_kp, y_ki, y_kd, -0.5, 0.5)  # Distance control
        
        # Calculate ground distance from shooting distance
        self.target_ground_distance = self.target_distance * math.cos(math.radians(self.shooting_angle))
        
        # ROS Publishers
        self.detection_pub = self.create_publisher(String, '/basket_detection', 10)
        self.distance_pub = self.create_publisher(Float32, '/basket_distance', 10)
        self.value_x_pub = self.create_publisher(Float32, '/value_x', 10)
        self.value_y_pub = self.create_publisher(Float32, '/value_y', 10)
        
        # Initialize cv_bridge
        self.bridge = None
        if CV_BRIDGE_AVAILABLE:
            try:
                self.bridge = CvBridge()
                self.image_pub = self.create_publisher(Image, '/detection_image', 10)
                self.get_logger().info("✅ CvBridge initialized")
            except Exception as e:
                self.get_logger().error(f"❌ CvBridge initialization failed: {e}")
        
        # Initialize window for direct display
        self.window_name = "Basketball Detection"
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        self.get_logger().info(f"✅ OpenCV window created: '{self.window_name}'")
        
        # Initialize RealSense
        self.pipeline = None
        self.align = None
        
        # Initialize YOLO model
        self.model = None
        
        # Set up components
        if REALSENSE_AVAILABLE:
            self.setup_realsense()
        else:
            self.get_logger().error("RealSense not available. Cannot proceed.")
            
        if YOLO_AVAILABLE:
            self.load_model()
        else:
            self.get_logger().error("YOLO not available. Cannot proceed.")
            
        # Only start timer if both camera and model are ready
        if self.pipeline and self.model:
            # Start detection thread
            self.create_timer(0.033, self.detection_callback)  # ~30 FPS
            self.get_logger().info("🚀 Detection loop started")
            self.get_logger().info(f"🎯 Target ground distance: {self.target_ground_distance:.3f}m")
        else:
            self.get_logger().error("Cannot start detection loop - missing components")
            
    def setup_realsense(self):
        """Setup RealSense camera pipeline"""
        try:
            self.get_logger().info("Initializing RealSense camera...")
            self.pipeline = rs.pipeline()
            config = rs.config()
            
            # Enable streams
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            
            # Start pipeline
            profile = self.pipeline.start(config)
            
            # Print camera info
            device = profile.get_device()
            self.get_logger().info(f"Connected to: {device.get_info(rs.camera_info.name)}")
            
            # Create align object
            self.align = rs.align(rs.stream.color)
            
            # Wait for camera to stabilize
            for i in range(30):  # Warm-up frames
                self.pipeline.wait_for_frames()
                if (i+1) % 10 == 0:
                    self.get_logger().info(f"Warm-up: {i+1}/30 frames")
                time.sleep(0.01)
                
            self.get_logger().info("✅ RealSense camera ready")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ RealSense setup failed: {str(e)}")
            return False
    
    def load_model(self):
        """Load YOLO model"""
        try:
            self.get_logger().info(f"Loading YOLO model from {self.model_path}")
            
            if not os.path.exists(self.model_path):
                self.get_logger().error(f"❌ Model file not found: {self.model_path}")
                return False
                
            self.model = YOLO(self.model_path)
            
            # Warmup with blank image
            blank_image = np.zeros((480, 640, 3), dtype=np.uint8)
            self.model(blank_image, verbose=False)
            
            self.get_logger().info("✅ YOLO model loaded")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Model loading failed: {str(e)}")
            return False
    
    def detection_callback(self):
        """Main detection loop"""
        if not self.pipeline or not self.model:
            self.get_logger().warn("Missing components - skipping detection")
            return
            
        try:
            start_time = time.time()  # For FPS calculation
            
            # Wait for frames
            frames = self.pipeline.wait_for_frames(5000)  # 5 second timeout
            if not frames:
                self.get_logger().warn("Timeout waiting for frames")
                return
                
            aligned_frames = self.align.process(frames)
            
            # Get aligned frames
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                self.get_logger().warn("Invalid frames received")
                return
                
            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            
            # Run detection
            results = self.model(color_image, verbose=False)
            
            # Process results
            result = results[0]
            
            # Create output image
            output_image = color_image.copy()
            
            # Draw bounding boxes and labels
            baskets_detected = 0
            best_detection = None
            best_confidence = 0
            
            for box in result.boxes:
                # Get box coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # Get confidence and class
                conf = float(box.conf[0])
                class_id = int(box.cls[0])
                
                # Only show basketball hoops (class 0) with good confidence
                if conf >= self.confidence_threshold and class_id == 0:
                    baskets_detected += 1
                    
                    # Calculate center point
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    
                    # Calculate width and height
                    width = x2 - x1
                    height = y2 - y1
                    
                    # Get distance at center point with averaging
                    distances = []
                    sample_size = 5
                    for dx in range(-sample_size, sample_size + 1):
                        for dy in range(-sample_size, sample_size + 1):
                            px = center_x + dx
                            py = center_y + dy
                            if 0 <= px < 640 and 0 <= py < 480:
                                d = depth_frame.get_distance(px, py)
                                if d > 0:  # Valid distance
                                    distances.append(d)
                    
                    if distances:
                        # Get median distance (more robust than mean)
                        distances.sort()
                        if len(distances) > 3:  # Remove outliers
                            distances = distances[1:-1]
                        distance = sum(distances) / len(distances)
                        
                        # Calculate ground distance using shooting angle
                        ground_distance = distance #* math.cos(math.radians(self.shooting_angle))
                        
                        # Calculate horizontal position - MODIFIED to be positive on left side
                        # Original: horizontal_position = (center_x - 320) / 320.0  # -1 to 1, negative on left
                        horizontal_position = -1 * (center_x - 320) / 320.0  # -1 to 1, positive on left
                        
                        # Calculate error from target distance
                        distance_error = ground_distance - self.target_ground_distance
                        
                        # Track best detection
                        if conf > best_confidence:
                            best_confidence = conf
                            best_detection = {
                                'confidence': conf,
                                'distance': distance,
                                'ground_distance': ground_distance,
                                'horizontal_position': horizontal_position,
                                'distance_error': distance_error,
                                'center': (center_x, center_y),
                                'box': (x1, y1, x2, y2)
                            }
                        
                        # Draw bounding box
                        color = (0, 255, 0) if abs(distance_error) < 0.1 else (0, 165, 255)
                        cv2.rectangle(output_image, (x1, y1), (x2, y2), color, 2)
                        
                        # Draw center point
                        cv2.circle(output_image, (center_x, center_y), 5, (0, 0, 255), -1)
                        
                        # Create distance label
                        distance_text = f"D: {distance:.2f}m G: {ground_distance:.2f}m"
                        error_text = f"Error: {distance_error:.3f}m"
                        horiz_text = f"Horiz: {horizontal_position:.2f}"
                        
                        # Draw text labels with background for readability
                        y_offset = y1 - 10
                        for text in [f"Basket {conf:.2f}", distance_text, error_text, horiz_text]:
                            text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                            cv2.rectangle(output_image, 
                                         (x1, y_offset - text_size[1] - 2),
                                         (x1 + text_size[0], y_offset + 2),
                                         (0, 0, 0), -1)
                            cv2.putText(output_image, text, (x1, y_offset),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                            y_offset -= 20
                        
                        # Publish ROS data
                        detection_msg = (f"BASKET,conf:{conf:.2f},dist:{distance:.3f},"
                                       f"ground_dist:{distance_error:.3f},horiz:{horizontal_position:.2f}")
                        self.detection_pub.publish(String(data=detection_msg))
                        
                        # Publish distance
                        self.distance_pub.publish(Float32(data=ground_distance))
                        
                        # Log periodically (not every frame to avoid flooding)
                        if time.time() % 1 < 0.1:  # Log approximately once per second
                            self.get_logger().info(f"🏀 Basket at ({center_x},{center_y}) - Distance: {distance:.2f}m")
            
            # Process PID control if basket detected
            if best_detection:
                # Calculate PID outputs
                value_x = self.x_pid.compute(best_detection['horizontal_position'])
                value_y = self.y_pid.compute(best_detection['distance_error'])
                
                # Publish PID outputs
                self.value_x_pub.publish(Float32(data=value_x))
                self.value_y_pub.publish(Float32(data=value_y))
                
                # Add PID info to visualization
                pid_text_x = f"PID X: {value_x:.3f}"
                pid_text_y = f"PID Y: {value_y:.3f}"
                
                # Log PID values periodically
                if time.time() % 1 < 0.1:
                    self.get_logger().info(f"PID Control: X={value_x:.3f}, Y={value_y:.3f}")
            else:
                # No detection - reset PIDs to avoid integral windup
                self.x_pid.reset()
                self.y_pid.reset()
                
                # Publish zeros when no detection
                self.value_x_pub.publish(Float32(data=0.0))
                self.value_y_pub.publish(Float32(data=0.0))
            
            # Add status overlay
            # Background for text
            status_background_height = 80  # Increased to fit PID info
            overlay = output_image.copy()
            cv2.rectangle(overlay, (0, 0), (640, status_background_height), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.7, output_image, 0.3, 0, output_image)
            
            # Add detection count and FPS
            fps = 1.0 / (time.time() - start_time)
            status_text = f"Detected: {baskets_detected} baskets | FPS: {fps:.1f}"
            cv2.putText(output_image, status_text, (10, 25), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Add position status
            if best_detection:
                error = best_detection['distance_error']
                horiz = best_detection['horizontal_position']
                
                # Add PID control values to display
                pid_text = f"PID Control: X={self.x_pid.compute(horiz):.3f}, Y={self.y_pid.compute(error):.3f}"
                cv2.putText(output_image, pid_text, (10, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                if abs(error) < 0.1 and abs(horiz) < 0.15:
                    position_status = "✅ POSITIONED CORRECTLY"
                    status_color = (0, 255, 0)  # Green
                else:
                    position_status = "⚠️ ADJUSTING POSITION" 
                    status_color = (0, 165, 255)  # Orange
                
                cv2.putText(output_image, position_status, (10, 75), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
                
                # Add target distance reference
                ref_text = f"Target: {self.target_ground_distance:.2f}m"
                cv2.putText(output_image, ref_text, (500, 75),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Display image directly in OpenCV window
            cv2.imshow(self.window_name, output_image)
            cv2.waitKey(1)  # Process window events
            
            # Publish ROS image if bridge is available
            if self.bridge and hasattr(self, 'image_pub'):
                try:
                    output_msg = self.bridge.cv2_to_imgmsg(output_image, "bgr8")
                    self.image_pub.publish(output_msg)
                except Exception as e:
                    self.get_logger().debug(f"Image publishing error: {e}")
            
        except Exception as e:
            self.get_logger().error(f"Detection error: {str(e)}")
            
            # Show error message in window
            try:
                error_img = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(error_img, "Detection Error", (180, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.putText(error_img, str(e), (50, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.imshow(self.window_name, error_img)
                cv2.waitKey(1)
            except:
                pass
    
    def destroy_node(self):
        """Cleanup resources on shutdown"""
        # Close OpenCV window
        cv2.destroyAllWindows()
        self.get_logger().info("OpenCV windows closed")
        
        # Stop RealSense pipeline
        if self.pipeline:
            try:
                self.pipeline.stop()
                self.get_logger().info("📷 RealSense pipeline stopped")
            except Exception as e:
                self.get_logger().error(f"Error stopping pipeline: {e}")
                
        super().destroy_node()

def main(args=None):
    # Initialize ROS
    rclpy.init(args=args)
    
    try:
        # Create node
        detector = RealsenseBasketDetector()
        
        # Spin node
        try:
            rclpy.spin(detector)
        except KeyboardInterrupt:
            detector.get_logger().info("Shutting down...")
        finally:
            # Clean up node
            detector.destroy_node()
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        # Always shutdown ROS
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"Error during shutdown: {e}")

if __name__ == '__main__':
    main()