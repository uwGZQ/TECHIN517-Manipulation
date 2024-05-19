#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from ros_sam.srv import Segmentation as SegmentationSrv, SegmentationRequest as SegmentationRequestMsg

from ros_sam import SAMClient

class ImageProcessor:
    def __init__(self):
        rospy.init_node('image_processor')
        self.bridge = CvBridge()
        self.sam = SAMClient('ros_sam')
        rospy.loginfo('Waiting for SAM service...')
        rospy.wait_for_service('ros_sam/segment')
        rospy.loginfo('Found SAM service')
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.masked_image_pub = rospy.Publisher('/masked_images', Image, queue_size=10)

    def generate_uniform_points(self, image, grid_size=100):
        points = []
        height, width, _ = image.shape
        for y in range(grid_size//2, height, grid_size):
            for x in range(grid_size//2, width, grid_size):
                points.append([x, y])
        return np.array(points)

    def image_callback(self, data):
        rospy.loginfo("Received an image, processing...")
        try:

            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            points = self.generate_uniform_points(cv_image, grid_size=100)  # Change grid size as needed
            labels = np.zeros(len(points))  # Assuming all points are initially negative samples
            boxes = np.asarray([[10, 10, 1700, 1300]])  # Example box

            masks, scores = self.sam.segment(cv_image, points, labels, boxes=boxes)

            for mask in masks:
                masked_image = cv2.addWeighted(cv_image, 1, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), 0.5, 0)
                masked_msg = self.bridge.cv2_to_imgmsg(masked_image, "bgr8")
                self.masked_image_pub.publish(masked_msg)
            rospy.loginfo("Masked image published")

        except Exception as e:
            rospy.logerr("Failed to process image: %s", e)

if __name__ == '__main__':
    ip = ImageProcessor()
    rospy.spin()
