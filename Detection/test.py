#!/usr/bin/env python3
# File: /workspace/Robocon25_codes/Detection/test_yolo.py

import pyrealsense2 as rs
import numpy as np
import cv2
import time
import os
import sys

# Add better error handling for imports
try:
    from ultralytics import YOLO
    print("Successfully imported ultralytics YOLO")
except ImportError as e:
    print(f"Error importing ultralytics: {e}")
    print("Trying to install ultralytics...")
    os.system("pip install ultralytics")
    try:
        from ultralytics import YOLO
    except ImportError:
        print("Failed to import ultralytics even after installation attempt")
        print("Please run: pip install ultralytics")
        sys.exit(1)

print("Starting RealSense + YOLO detection...")

# Initialize RealSense
pipeline = rs.pipeline()
config = rs.config()
print("Configuring RealSense streams...")
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Check if model file exists
model_path = "last.pt"
if not os.path.exists(model_path):
    print(f"ERROR: Model file '{model_path}' not found!")
    print("Looking for alternative model files...")
    pt_files = [f for f in os.listdir('.') if f.endswith('.pt')]
    if pt_files:
        model_path = pt_files[0]
        print(f"Using alternative model file: {model_path}")
    else:
        print("No .pt model files found. Using default YOLO model.")
        model_path = "yolov8n.pt"  # Use default model

# Start streaming with explicit error handling
try:
    print("Starting RealSense pipeline...")
    pipeline.start(config)
    print("Pipeline started successfully!")
except Exception as e:
    print(f"ERROR starting RealSense pipeline: {e}")
    sys.exit(1)

# Load YOLO model with error handling
try:
    print(f"Loading YOLO model from: {model_path}")
    model = YOLO(model_path)
    print("YOLO model loaded successfully!")
except Exception as e:
    print(f"ERROR loading YOLO model: {e}")
    pipeline.stop()
    sys.exit(1)

# Configure depth stream post-processing
align = rs.align(rs.stream.color)

print("Beginning detection loop. Press ESC to exit.")
try:
    while True:
        # Wait for frames
        frames = pipeline.wait_for_frames()
        
        # Align depth frame to color frame
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        
        if not color_frame or not depth_frame:
            print("Waiting for valid frames...")
            continue
            
        # Convert to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        
        # Run YOLO detection
        try:
            results = model(color_image)
            
            # Process results
            result = results[0]
            
            # Create a copy for drawing
            output_image = color_image.copy()
            
            # Draw bounding boxes and labels
            for box in result.boxes:
                # Get box coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # Get confidence
                conf = float(box.conf[0])
                
                # Get class ID and name
                class_id = int(box.cls[0])
                class_name = result.names[class_id]
                
                # Only show detections with confidence > 0.
                if conf > 0.:
                    # Draw bounding box
                    cv2.rectangle(output_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    # Calculate center point
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    
                    # Get distance at center point
                    distance = depth_frame.get_distance(center_x, center_y)
                    
                    # Draw label with confidence and distance
                    label = f"{class_name} {conf:.2f}, {distance:.2f}m"
                    cv2.putText(output_image, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 
                               0.6, (0, 255, 0), 2)
                    
                    # Print detection for debugging
                    print(f"Detected {class_name} with confidence {conf:.2f} at distance {distance:.2f}m")
                    
            # Show the result
            cv2.imshow("YOLO + RealSense", output_image)
            
        except Exception as e:
            print(f"Error during detection: {e}")
            # Show original image if detection fails
            cv2.imshow("YOLO + RealSense", color_image)
        
        # Break loop with ESC
        key = cv2.waitKey(1)
        if key == 27:  # ESC
            print("ESC pressed, exiting...")
            break
            
except KeyboardInterrupt:
    print("Interrupted by user")
except Exception as e:
    print(f"Error in main loop: {e}")
finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
    print("Detection stopped.")