#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
from ultralytics_ros.msg import YoloResult
from vision_msgs.msg import Detection2DArray, Detection2D

def depth_callback(depth_data, args):
    camera_info_msg, detections = args
    bridge = CvBridge()
    depth_image = bridge.imgmsg_to_cv2(depth_data, desired_encoding='passthrough')

    # Camera intrinsic parameters
    fx = camera_info_msg.K[0]  # Focal length in x-axis
    fy = camera_info_msg.K[4]  # Focal length in y-axis
    cx = camera_info_msg.K[2]  # Optical center x-coordinate
    cy = camera_info_msg.K[5]  # Optical center y-coordinate

    # Process each detection
    for detection in detections:
        bbox = detection.bbox
        x_center = bbox.center.x
        y_center = bbox.center.y

        # Ensure the center points are within the image dimensions
        if 0 <= int(y_center) < depth_image.shape[0] and 0 <= int(x_center) < depth_image.shape[1]:
            # Get the depth value at the bbox center
            depth = depth_image[int(y_center), int(x_center)] / 1000.0  # Convert from millimeters to meters

            # Convert pixel coordinates to normalized image coordinates
            x_normalized = (x_center - cx) / fx
            y_normalized = (y_center - cy) / fy

            # Convert to camera coordinates
            x_camera = x_normalized * depth
            y_camera = y_normalized * depth
            z_camera = depth

            print(f"Object ID: {detection.results[0].id} - Camera Coordinates: x={x_camera:.3f}, y={y_camera:.3f}, z={z_camera:.3f}")
        else:
            print("Bounding box center is out of image bounds.")

    print("\n")
    rospy.sleep(1.0)

def yolo_callback(data):
    camera_info_msg = rospy.wait_for_message("/camera/aligned_depth_to_color/camera_info", CameraInfo)
    depth_topic = "/camera/aligned_depth_to_color/image_raw"
    rospy.Subscriber(depth_topic, Image, depth_callback, (camera_info_msg, data.detections.detections))

def listener():
    rospy.init_node('pixel_to_camera_converter', anonymous=True)
    rospy.Subscriber("/yolo_result", YoloResult, yolo_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
