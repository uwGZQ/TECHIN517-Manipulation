import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def lookup_transform(tf_buffer, from_frame, to_frame):
    try:
        # We use rospy.Time(0) to get the latest available transform
        return tf_buffer.lookup_transform(from_frame, to_frame, rospy.Time(0), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr("TF2 Exception: %s" % str(e))
        return None

def main():
    rospy.init_node('tf_example_node')
    
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    rate = rospy.Rate(1.0)  # 1 Hz
    while not rospy.is_shutdown():
        transform = lookup_transform(tf_buffer, 'base_link', 'aruco_marker_frame')
        if transform:
            translation = transform.transform.translation
            #publish static transform base and aruco 
            #ros tf understand the diff, tf to understand the transform b/
            rotation = transform.transform.rotation
            rospy.loginfo("Translation: x=%f, y=%f, z=%f" % (translation.x, translation.y, translation.z))
            rospy.loginfo("Rotation: x=%f, y=%f, z=%f, w=%f" % (rotation.x, rotation.y, rotation.z, rotation.w))
        rate.sleep()

if __name__ == '__main__':
    main()
