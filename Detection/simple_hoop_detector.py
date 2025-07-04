#!/usr/bin/env python3
# File: /workspace/Robocon25_codes/Detection/simple_hoop_detector.py

import pyrealsense2 as rs
import numpy as np
import cv2
import time

# Print debug info
print("Starting simplified hoop detector...")
print("OpenCV version:", cv2.__version__)

# Configure RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()

# Print available devices
ctx = rs.context()
devices = ctx.query_devices()
print(f"Found {len(devices)} RealSense devices:")
for i, dev in enumerate(devices):
    print(f"  Device {i}: {dev.get_info(rs.camera_info.name)}")

# Configure streams - use lower resolution for better performance
print("Configuring streams...")
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start pipeline with error handling
try:
    print("Starting pipeline...")
    pipeline.start(config)
    print("Pipeline started successfully.")
except Exception as e:
    print(f"Error starting pipeline: {e}")
    exit(1)

# Allow auto-exposure to stabilize
print("Allowing camera to stabilize...")
for i in range(10):
    pipeline.wait_for_frames()

# Orange color range for hoop detection - WIDE range to start
lower_orange = np.array([5, 50, 50])   # Lower HSV threshold
upper_orange = np.array([30, 255, 255]) # Upper HSV threshold

print("Beginning detection loop. Press ESC to exit.")
print("Debug info will be printed to console.")

try:
    frame_count = 0
    while True:
        frame_count += 1
        if frame_count % 10 == 0:  # Print status every 10 frames
            print(f"Processing frame {frame_count}...")
        
        # Get frameset of depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            print("Waiting for valid frames...")
            continue
            
        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        
        # Convert to HSV
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        
        # Create a mask for orange color
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        
        # Clean up mask with morphological operations
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Create a copy for visualization
        result_image = color_image.copy()
        
        # Draw the mask in a corner for debugging
        small_mask = cv2.resize(mask, (160, 120))
        small_mask = cv2.cvtColor(small_mask, cv2.COLOR_GRAY2BGR)
        result_image[0:120, 0:160] = small_mask
        
        # Draw text showing the HSV range used
        cv2.putText(result_image, f"H: {lower_orange[0]}-{upper_orange[0]}", 
                   (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(result_image, f"S: {lower_orange[1]}-{upper_orange[1]}", 
                   (10, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(result_image, f"V: {lower_orange[2]}-{upper_orange[2]}", 
                   (10, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Process contours
        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            # Only process if area is significant
            if area > 300:  # Lower threshold to detect more
                # Draw all contours for debugging
                cv2.drawContours(result_image, [largest_contour], -1, (0, 255, 0), 2)
                
                # Get bounding box
                x, y, w, h = cv2.boundingRect(largest_contour)
                cv2.rectangle(result_image, (x, y), (x+w, y+h), (0, 0, 255), 2)
                
                # Calculate center
                center_x = x + w//2
                center_y = y + h//2
                center = (center_x, center_y)
                
                # Get distance
                distance = depth_frame.get_distance(center_x, center_y)
                
                # Display info
                cv2.circle(result_image, center, 5, (255, 0, 0), -1)
                cv2.putText(result_image, f"Area: {int(area)}", 
                           (x, y-30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(result_image, f"Dist: {distance:.2f}m", 
                           (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Print detection info every 30 frames
                if frame_count % 30 == 0:
                    print(f"Detected object: Area={int(area)}, Distance={distance:.2f}m")
        
        # Show images
        cv2.namedWindow('Hoop Detector', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Hoop Detector', result_image)
        
        # Break loop with ESC key
        key = cv2.waitKey(1)
        if key == 27:  # ESC key
            print("ESC pressed. Exiting...")
            break
            
except Exception as e:
    print(f"Error in detection loop: {e}")

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
    print("Detection stopped.")