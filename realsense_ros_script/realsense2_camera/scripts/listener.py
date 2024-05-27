#!/usr/bin/env python
import rospy
from realsense2_camera.msg import DetectedObject, DetectedObjectArray

def detected_objects_callback(msg):
    rospy.loginfo("Received detected objects:")
    for obj in msg.objects:
        rospy.loginfo("Object Name: %s", obj.object_name)
        rospy.loginfo("Object Position: x=%.4f, y=%.4f, z=%.4f", obj.position.x, obj.position.y, obj.position.z)
    rospy.loginfo("-------------------------------------------------------------")

def listener():
    rospy.init_node('detected_objects_listener', anonymous=True)
    rospy.Subscriber("/detected_objects", DetectedObjectArray, detected_objects_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
