#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image

import cv_bridge
import numpy as np
import roslib.packages
import rospy
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from ultralytics_ros.msg import YoloResult

def callback(data):
    print("Header:")
    # print("  seq: ", data.header.seq)
    # print("  stamp: ", data.header.stamp)
    print("  frame_id: ", data.header.frame_id)
    
    print("Detections:")
    for detection in data.detections.detections:
        # print(len(detection.results))
        print("  ID: ", detection.results[0].id)
        print("  Score: ", detection.results[0].score)
        print("  Bounding Box Center: ")
    #     print("  Bounding Box: ", detection.bbox)
        print(detection.bbox.center.x, detection.bbox.center.y)
        print("  Bounding Box: ", detection.bbox)

def callback_came_info(data):
    print("Header:")
    print("  seq: ", data.header.seq)
    # print("  stamp: ", data.header.stamp)
    print("  frame_id: ", data.header.frame_id)
    
    print("Detections:")
    for detection in data.detections.detections:
    #     print("  ID: ", detection.results[0].id)
    #     print("  Score: ", detection.results[0].score)
    #     print("  Bounding Box Center: ", detection.bbox.center)
    #     print("  Bounding Box: ", detection.bbox)
        print(detection.bbox.center.x, detection.bbox.center.y)
    
def listener():
    rospy.init_node('yoloresult_listener', anonymous=True)
    rospy.Subscriber("/yolo_result", YoloResult, callback)

    # rospy.init_node('camera_info_listener', anonymous=True)
    # rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", YoloResult, callback_came_info)
    
    rospy.spin()

if __name__ == '__main__':
    listener()
