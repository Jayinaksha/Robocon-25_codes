#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String

# Try importing YOLO
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    print("⚠️ YOLO not available - install with: pip install ultralytics")
    YOLO_AVAILABLE = False

class BasketballDetector(Node):
    def __init__(self):
        super().__init__('basketball_detector')
        
        # Parameters
        self.declare_parameter('model_path', '/workspace/Robocon25_codes/Detection/last.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('use_compressed', False)
        
        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.image_topic = self.get_parameter('image_topic').value
        self.use_compressed = self.get_parameter('use_compressed').value
        
        # Initialize CV Bridge
        self.cv_bridge = CvBridge()
        
        # Publishers
        self.detection_pub = self.create_publisher(String, '/basketball_detection', 10)
        self.image_pub = self.create_publisher(Image, '/detection_image', 10)
        
        # Initialize model
        self.model = None
        if YOLO_AVAILABLE:
            self.init_model()
        else:
            self.get_logger().error("YOLO is not available. Cannot perform detection.")
        
        # Subscribers
        if self.use_compressed:
            self.image_sub = self.create_subscription(
                CompressedImage,
                f"{self.image_topic}/compressed",
                self.compressed_image_callback,
                10)
        else:
            self.image_sub = self.create_subscription(
                Image,
                self.image_topic,
                self.image_callback,
                10)
        
        self.get_logger().info("✅ Basketball Detector initialized")
        self.get_logger().info(f"📷 Subscribed to: {self.image_topic}")
        self.get_logger().info(f"🧠 Model: {self.model_path}")
    
    def init_model(self):
        """Initialize YOLO model"""
        try:
            if not os.path.exists(self.model_path):
                self.get_logger().error(f"❌ Model file not found: {self.model_path}")
                return False
                
            self.get_logger().info("Loading YOLO model...")
            self.model = YOLO(self.model_path)
            
            # Warmup the model with a blank image
            blank_image = np.zeros((480, 640, 3), dtype=np.uint8)
            self.model(blank_image, verbose=False)
            
            self.get_logger().info("✅ YOLO model loaded successfully")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Model initialization failed: {str(e)}")
            return False
    
    def image_callback(self, msg):
        """Process raw image messages"""
        if not self.model:
            return
        
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_image(cv_image)
        except Exception as e:
            self.get_logger().error(f"Image processing error: {str(e)}")
    
    def compressed_image_callback(self, msg):
        """Process compressed image messages"""
        if not self.model:
            return
        
        try:
            # Convert compressed image to OpenCV format
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.process_image(cv_image)
        except Exception as e:
            self.get_logger().error(f"Compressed image processing error: {str(e)}")
    
    def process_image(self, cv_image):
        """Process image with YOLO model"""
        try:
            # Run detection
            results = self.model(cv_image, verbose=False)
            
            # Create visualization image (copy of original)
            vis_image = cv_image.copy()
            
            # Process detection results
            detections = []
            
            for result in results:
                if result.boxes is not None:
                    for box in result.boxes:
                        confidence = float(box.conf[0])
                        class_id = int(box.cls[0])
                        
                        # Only consider basketball hoops (class 0) with confidence above threshold
                        if confidence > self.confidence_threshold and class_id == 0:
                            # Get bounding box coordinates
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            center_x = (x1 + x2) // 2
                            center_y = (y1 + y2) // 2
                            
                            # Add to detections list
                            detections.append({
                                'confidence': confidence,
                                'class_id': class_id,
                                'box': (x1, y1, x2, y2),
                                'center': (center_x, center_y)
                            })
                            
                            # Draw bounding box and center point
                            cv2.rectangle(vis_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            cv2.circle(vis_image, (center_x, center_y), 5, (0, 0, 255), -1)
                            
                            # Add confidence text
                            label = f"Basket: {confidence:.2f}"
                            cv2.putText(vis_image, label, (x1, y1 - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Add text with detection count
            count_text = f"Detected: {len(detections)} baskets"
            cv2.putText(vis_image, count_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Publish detection results
            if detections:
                best_detection = max(detections, key=lambda d: d['confidence'])
                x1, y1, x2, y2 = best_detection['box']
                cx, cy = best_detection['center']
                
                # Create detection message with main info
                msg_text = (f"DETECTION,confidence:{best_detection['confidence']:.2f},"
                           f"center_x:{cx},center_y:{cy},width:{x2-x1},height:{y2-y1}")
                self.detection_pub.publish(String(data=msg_text))
                
                self.get_logger().info(f"🏀 Detected basket at ({cx}, {cy}) with confidence {best_detection['confidence']:.2f}")
            
            # Publish visualization image
            self.publish_image(vis_image)
            
        except Exception as e:
            self.get_logger().error(f"Detection error: {str(e)}")
    
    def publish_image(self, image):
        """Publish OpenCV image as ROS Image message"""
        try:
            ros_image = self.cv_bridge.cv2_to_imgmsg(image, "bgr8")
            self.image_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"Image publishing error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    detector = BasketballDetector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        detector.get_logger().info("🛑 Shutting down Basketball Detector...")
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()