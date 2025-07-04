#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import cv2
import time

print("RealSense D455 Camera Test")
print("=========================")

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

print(f"Using device: {device.get_info(rs.camera_info.name)}")
print(f"Serial number: {device.get_info(rs.camera_info.serial_number)}")

# Enable streams
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    # Wait for frames
    for i in range(30):  # Show 30 frames
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), 
            cv2.COLORMAP_JET
        )

        # Show distance to center of image
        dist_to_center = depth_frame.get_distance(
            depth_image.shape[1]//2, 
            depth_image.shape[0]//2
        )
        print(f"Frame {i+1}: Distance to center: {dist_to_center:.2f}m")

        # Display images side by side
        images = np.hstack((color_image, depth_colormap))
        cv2.namedWindow('D455: Color + Depth', cv2.WINDOW_NORMAL)
        cv2.imshow('D455: Color + Depth', images)
        if cv2.waitKey(1) == 27:  # ESC key
            break
        time.sleep(0.1)

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
