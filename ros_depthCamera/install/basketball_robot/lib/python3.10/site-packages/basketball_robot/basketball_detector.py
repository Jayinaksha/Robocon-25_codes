#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String
import threading
import time
import os
import sys

# Import libraries directly (bypassing cv_bridge)
try:
    import pyrealsense2 as rs
    import numpy as np
    import cv2
    from ultralytics import YOLO
    DIRECT_MODE = True
    print("Successfully loaded direct vision libraries")
except ImportError as e:
    print(f"Warning: Could not import required libraries: {e}")
    print("Falling back to simulation mode")
    DIRECT_MODE = False
    import random

class BasketballDetector(Node):
    def __init__(self):
        super().__init__('basketball_detector')
        
        # Initialize running flag early
        self.running = True
        
        # Parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('detection_classes', ['sports ball', 'person'])
        self.declare_parameter('detection_threshold', 0.5)
        self.declare_parameter('target_distance', 2.5)
        
        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.detection_classes = self.get_parameter('detection_classes').value
        self.detection_threshold = self.get_parameter('detection_threshold').value
        self.target_distance = self.get_parameter('target_distance').value
        
        # Publishers for detection results
        self.distance_pub = self.create_publisher(Float32, '/basketball/distance', 10)
        self.x_offset_pub = self.create_publisher(Float32, '/basketball/x_offset', 10)
        self.detected_pub = self.create_publisher(Bool, '/basketball/detected', 10)
        self.debug_pub = self.create_publisher(String, '/basketball/debug', 10)
        
        # Initialize state variables BEFORE starting threads
        self.current_distance = self.target_distance
        self.current_offset = 0.0
        self.target_detected = False
        self.vision_initialized = False
        
        # If direct mode is available, initialize vision system
        if DIRECT_MODE:
            self.get_logger().info('Initializing direct vision system...')
            self.init_vision()
            
            # Start detection thread
            if self.vision_initialized:
                self.detection_thread = threading.Thread(target=self.detection_loop)
                self.detection_thread.daemon = True
                self.detection_thread.start()
                self.get_logger().info('Detection thread started')
            else:
                self.get_logger().error('Vision system initialization failed, falling back to simulation')
                self.start_simulation_mode()
        else:
            self.get_logger().warn('Direct vision mode not available, using simulation')
            # Start simulation thread
            self.start_simulation_mode()
            
        # For publishing data
        self.create_timer(0.1, self.publish_detection)
        
        self.get_logger().info('Basketball detector initialized')

    def start_simulation_mode(self):
        """Initialize and start the simulation thread"""
        self.simulation_thread = threading.Thread(target=self.simulation_loop)
        self.simulation_thread.daemon = True
        self.simulation_thread.start()
        self.get_logger().info('Simulation thread started')

    def init_vision(self):
        """Initialize RealSense and YOLO directly"""
        try:
            # Initialize RealSense pipeline
            self.pipeline = rs.pipeline()
            config = rs.config()
            self.get_logger().info('Configuring RealSense streams...')
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            
            # Start streaming
            self.get_logger().info('Starting RealSense pipeline...')
            self.pipeline.start(config)
            
            # Configure depth stream post-processing
            self.align = rs.align(rs.stream.color)
            
            # Check if model file exists
            if not os.path.exists(self.model_path):
                self.get_logger().warn(f"Model {self.model_path} not found, using default model")
                self.model_path = "yolov8n.pt"  # Use default model
            
            # Load YOLO model
            self.get_logger().info(f'Loading YOLO model from {self.model_path}...')
            self.model = YOLO(self.model_path)
            
            # Create window for display
            cv2.namedWindow("Basketball Detection", cv2.WINDOW_NORMAL)
            
            self.vision_initialized = True
            self.get_logger().info('Vision system initialized successfully')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize vision system: {e}')
            self.vision_initialized = False
            return False

    def simulation_loop(self):
        """Simulate detection (fallback when vision system is unavailable)"""
        if not DIRECT_MODE:
            import random  # Local import for simulation mode only
        
        detection_toggle_time = time.time() + random.uniform(5, 10)
        
        while self.running:
            current_time = time.time()
            
            # Toggle detection status periodically
            if current_time > detection_toggle_time:
                self.target_detected = not self.target_detected
                detection_toggle_time = current_time + random.uniform(5, 15)
                
                status = "DETECTED" if self.target_detected else "LOST"
                self.get_logger().info(f"Target {status} (simulation)")
            
            # If target is detected, simulate some movement
            if self.target_detected:
                # Simulate distance with some noise
                self.current_distance = self.target_distance + random.uniform(-0.5, 0.5)
                
                # Simulate horizontal offset
                self.current_offset = random.uniform(-0.3, 0.3)
            
            time.sleep(0.05)

    def detection_loop(self):
        """Main detection loop using RealSense and YOLO directly"""
        while self.running and self.vision_initialized:
            try:
                # Wait for frames
                frames = self.pipeline.wait_for_frames()
                
                # Align depth frame to color frame
                aligned_frames = self.align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()
                
                if not color_frame or not depth_frame:
                    self.get_logger().warn("Missing frames, skipping detection")
                    continue
                    
                # Convert to numpy arrays - direct usage without cv_bridge
                color_image = np.asanyarray(color_frame.get_data())
                
                # Run YOLO detection
                results = self.model(color_image)
                result = results[0]
                
                # Create a copy for visualization
                viz_image = color_image.copy()
                
                # Process results
                best_conf = 0
                self.target_detected = False
                height, width = color_image.shape[:2]
                image_center_x = width / 2
                
                for box in result.boxes:
                    # Get box coordinates
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    
                    # Get confidence
                    conf = float(box.conf[0])
                    
                    # Get class ID and name
                    class_id = int(box.cls[0])
                    class_name = result.names[class_id]
                    
                    # Only process relevant classes with sufficient confidence
                    if conf > self.detection_threshold and class_name in self.detection_classes:
                        self.target_detected = True
                        
                        if conf > best_conf:
                            best_conf = conf
                            
                            # Calculate center point
                            center_x = (x1 + x2) // 2
                            center_y = (y1 + y2) // 2
                            
                            # Calculate horizontal offset from center (-1 to 1)
                            self.current_offset = (center_x - image_center_x) / image_center_x
                            
                            # Get distance
                            self.current_distance = depth_frame.get_distance(center_x, center_y)
                            
                            # Draw visualization
                            cv2.rectangle(viz_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            cv2.circle(viz_image, (center_x, center_y), 5, (0, 0, 255), -1)
                            cv2.line(viz_image, (int(image_center_x), center_y), (center_x, center_y), (0, 0, 255), 2)
                            
                            # Draw text with distance and offset
                            label = f"{class_name} {conf:.2f}, {self.current_distance:.2f}m, off:{self.current_offset:.2f}"
                            cv2.putText(viz_image, label, (x1, y1-10), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Add status text
                if self.target_detected:
                    status = f"TARGET DETECTED: Dist={self.current_distance:.2f}m, Offset={self.current_offset:.2f}"
                    cv2.putText(viz_image, status, (10, 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    cv2.putText(viz_image, "NO TARGET DETECTED", (10, 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # Display image
                cv2.imshow("Basketball Detection", viz_image)
                if cv2.waitKey(1) & 0xFF == 27:  # ESC to exit
                    self.get_logger().info("ESC pressed, stopping detection")
                    break
                
            except Exception as e:
                self.get_logger().error(f"Error in detection loop: {e}")
                time.sleep(1.0)  # Wait before retrying
    
    def publish_detection(self):
        """Publish current detection state"""
        # Publish distance
        distance_msg = Float32()
        distance_msg.data = float(self.current_distance)
        self.distance_pub.publish(distance_msg)
        
        # Publish horizontal offset
        offset_msg = Float32()
        offset_msg.data = float(self.current_offset)
        self.x_offset_pub.publish(offset_msg)
        
        # Publish detection status
        detected_msg = Bool()
        detected_msg.data = self.target_detected
        self.detected_pub.publish(detected_msg)
        
        # Publish debug message
        if self.target_detected:
            debug_msg = String()
            debug_msg.data = f"Detection: {self.current_distance:.2f}m, {self.current_offset:.2f}offset"
            self.debug_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BasketballDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        
        # Clean up resources
        if hasattr(node, 'pipeline') and node.vision_initialized:
            try:
                node.pipeline.stop()
                node.get_logger().info("RealSense pipeline stopped")
            except Exception as e:
                node.get_logger().error(f'Error stopping pipeline: {e}')
        
        # Close OpenCV windows
        if DIRECT_MODE:
            cv2.destroyAllWindows()
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()