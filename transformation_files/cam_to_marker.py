import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R

# Initialize the ROS node
rospy.init_node('cam_to_marker_node')

# Create a TF2 broadcaster
tf_broadcaster = tf2_ros.TransformBroadcaster()

# Callback function to receive the marker pose and publish the transformation
def marker_pose_callback(pose_msg):
    # Extract the marker's rotation and translation from the pose message
    marker_rotation_cam = np.array([
        pose_msg.pose.orientation.x,
        pose_msg.pose.orientation.y,
        pose_msg.pose.orientation.z,
        pose_msg.pose.orientation.w
    ])
    marker_translation_cam = np.array([
        pose_msg.pose.position.x,
        pose_msg.pose.position.y,
        pose_msg.pose.position.z
    ])

    # Create the transformation from camera to marker
    cam_to_marker_transform = TransformStamped()
    cam_to_marker_transform.header.stamp = rospy.Time.now()
    cam_to_marker_transform.header.frame_id = 'camera_frame'
    cam_to_marker_transform.child_frame_id = 'marker_frame'
    cam_to_marker_transform.transform.translation.x = marker_translation_cam[0]
    cam_to_marker_transform.transform.translation.y = marker_translation_cam[1]
    cam_to_marker_transform.transform.translation.z = marker_translation_cam[2]
    cam_to_marker_transform.transform.rotation.x = marker_rotation_cam[0]
    cam_to_marker_transform.transform.rotation.y = marker_rotation_cam[1]
    cam_to_marker_transform.transform.rotation.z = marker_rotation_cam[2]
    cam_to_marker_transform.transform.rotation.w = marker_rotation_cam[3]

    # Publish the transformation to the TF tree
    tf_broadcaster.sendTransform(cam_to_marker_transform)

# Subscribe to the marker pose topic
rospy.Subscriber('/aruco_single/pose', PoseStamped, marker_pose_callback)

# Spin the node
rospy.spin()