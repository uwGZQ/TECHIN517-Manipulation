import rospy
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R

# Initialize the ROS node
rospy.init_node('marker_to_base_link_node')

# Create a TF2 broadcaster
tf_broadcaster = tf2_ros.TransformBroadcaster()

# Define the transformation from marker to Kinova arm base_link (manually measured)
marker_to_base_link_translation = np.array([0.1, 0.2, 0.3])  # Example values, replace with actual measurements
marker_to_base_link_rotation = R.from_euler('xyz', [0.1, 0.2, 0.3]).as_quat()  # Example values, replace with actual measurements

# Create the transformation from marker to base_link
marker_to_base_link_transform = TransformStamped()
marker_to_base_link_transform.header.frame_id = 'marker_frame'
marker_to_base_link_transform.child_frame_id = 'base_link'
marker_to_base_link_transform.transform.translation.x = marker_to_base_link_translation[0]
marker_to_base_link_transform.transform.translation.y = marker_to_base_link_translation[1]
marker_to_base_link_transform.transform.translation.z = marker_to_base_link_translation[2]
marker_to_base_link_transform.transform.rotation.x = marker_to_base_link_rotation[0]
marker_to_base_link_transform.transform.rotation.y = marker_to_base_link_rotation[1]
marker_to_base_link_transform.transform.rotation.z = marker_to_base_link_rotation[2]
marker_to_base_link_transform.transform.rotation.w = marker_to_base_link_rotation[3]

# Publish the transformation continuously
rate = rospy.Rate(10)  # Publish at 10 Hz
while not rospy.is_shutdown():
    marker_to_base_link_transform.header.stamp = rospy.Time.now()
    tf_broadcaster.sendTransform(marker_to_base_link_transform)
    rate.sleep()