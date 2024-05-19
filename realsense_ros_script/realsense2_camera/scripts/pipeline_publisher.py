#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
from ultralytics_ros.msg import YoloResult
from vision_msgs.msg import Detection2DArray, Detection2D
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from tf2_ros import TransformBroadcaster
from realsense2_camera.msg import DetectedObject, DetectedObjectArray

# Define the mapping of object IDs to their names
object_names = {
    80: "socks",
    39: "bottle",
    81: "blanket"
}

def depth_callback(depth_data, args):
    camera_info_msg, detections = args
    bridge = CvBridge()
    depth_image = bridge.imgmsg_to_cv2(depth_data, desired_encoding='passthrough')

    global tf_broadcaster, detected_objects_pub

    # Camera intrinsic parameters
    fx = camera_info_msg.K[0] # Focal length in x-axis
    fy = camera_info_msg.K[4] # Focal length in y-axis
    cx = camera_info_msg.K[2] # Optical center x-coordinate
    cy = camera_info_msg.K[5] # Optical center y-coordinate

    # Create a DetectedObjectArray message to store the detected objects
    detected_objects_msg = DetectedObjectArray()
    detected_objects_msg.header.stamp = rospy.Time.now()
    detected_objects_msg.header.frame_id = "base_link"

    # Process each detection
    for d in detections:
        bbox = d.bbox
        x_center = bbox.center.x
        y_center = bbox.center.y

        # Ensure the center points are within the image dimensions
        if 0 <= int(y_center) < depth_image.shape[0] and 0 <= int(x_center) < depth_image.shape[1]:
            # Get the depth value at the bbox center
            depth = depth_image[int(y_center), int(x_center)] / 1000.0 # Convert from millimeters to meters

            # Convert pixel coordinates to camera coordinates
            x_camera = (x_center - cx) * depth / fx
            y_camera = (y_center - cy) * depth / fy
            z_camera = depth

            # Create a PointStamped message with the camera coordinates
            camera_point = geometry_msgs.msg.PointStamped()
            camera_point.header.frame_id = "camera_link"
            camera_point.header.stamp = rospy.Time.now()
            camera_point.point.x = x_camera
            camera_point.point.y = y_camera
            camera_point.point.z = z_camera

            # Transform the point from camera frame to Aruco marker frame
            try:
                aruco_point = tf_buffer.transform(camera_point, "aruco_marker_frame")
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("Failed to transform point from camera frame to Aruco marker frame")
                continue

            # Transform the point from Aruco marker frame to base link frame
            try:
                base_link_point = tf_buffer.transform(aruco_point, "base_link")
                object_id = d.results[0].id
                object_name = object_names.get(object_id, "Unknown")

                # Create a DetectedObject message for the current object
                detected_object_msg = DetectedObject()
                detected_object_msg.object_id = object_id
                detected_object_msg.object_name = object_name
                detected_object_msg.position.x = base_link_point.point.x
                detected_object_msg.position.y = base_link_point.point.y
                detected_object_msg.position.z = base_link_point.point.z

                print(f"object_name: {object_name} coordinate: {base_link_point.point}")

                # Append the DetectedObject message to the DetectedObjectArray
                detected_objects_msg.objects.append(detected_object_msg)

                # Broadcast the transformation
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "base_link"
                t.child_frame_id = f"object_{object_id}"
                t.transform.translation.x = base_link_point.point.x
                t.transform.translation.y = base_link_point.point.y
                t.transform.translation.z = base_link_point.point.z
                t.transform.rotation.x = 0.0  # Assuming no rotation; adjust if needed
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0
                tf_broadcaster.sendTransform(t)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("Failed to transform point from Aruco marker frame to base link frame")
                continue

    # Publish the DetectedObjectArray message
    print("-------------------------------------------------------------------")
    detected_objects_pub.publish(detected_objects_msg)
    rospy.sleep(0.1)

def yolo_callback(data):
    camera_info_msg = rospy.wait_for_message("/camera/aligned_depth_to_color/camera_info", CameraInfo)
    depth_topic = "/camera/aligned_depth_to_color/image_raw"
    rospy.Subscriber(depth_topic, Image, depth_callback, (camera_info_msg, data.detections.detections))

def listener():
    global tf_buffer, tf_broadcaster, detected_objects_pub
    rospy.init_node('pixel_to_camera_converter', anonymous=True)
    # tf2_ros.ConvertRegistration().add_to_registry(geometry_msgs.msg.PointStamped)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tf_broadcaster = TransformBroadcaster()
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectArray, queue_size=10)
    rospy.Subscriber("/yolo_result", YoloResult, yolo_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()