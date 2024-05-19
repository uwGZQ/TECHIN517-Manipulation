#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point as PointMsg
from std_msgs.msg import Int32MultiArray as Int32MultiArrayMsg
from ros_sam.srv import Segmentation as SegmentationSrv, SegmentationRequest as SegmentationRequestMsg

from ros_sam import SAMClient, show_mask

class ImageProcessor:
    def __init__(self):
        rospy.init_node('image_processor')
        self.bridge = CvBridge()

        # Waiting for SAM segmentation service
        rospy.loginfo('Waiting for SAM service...')
        rospy.wait_for_service('ros_sam/segment')
        rospy.loginfo('Found SAM service')

        # Set up SAM client
        self.sam = SAMClient('ros_sam')

        # Set up subscriber to the camera topic
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

        # Set up publisher for masked images
        self.masked_image_pub = rospy.Publisher('/masked_images', Image, queue_size=10)

    def image_callback(self, data):
        # Convert ROS Image message to OpenCV format
        rospy.loginfo("Received an image, processing...")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Define points, labels, and boxes
            points = np.array([[1035, 640], [1325, 610]])
            labels = np.array([0, 0])
            boxes = np.asarray([[54, 350, 1700, 1300]])

            # Perform segmentation
            masks, scores = self.sam.segment(cv_image, points, labels, boxes=boxes)

            # Create masked images and publish them
            for mask in masks:
                mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)  # Convert mask to BGR
                mask_color[mask == 0] = [0, 0, 0]  # Set background to black
                mask_color[mask == 1] = [0, 255, 0]  # Set masked area to green

                # Overlay the mask on the original image
                masked_image = cv2.addWeighted(cv_image, 1, mask_color, 0.5, 0)

                # Convert the masked image back to ROS Image message
                masked_msg = self.bridge.cv2_to_imgmsg(masked_image, "bgr8")
                self.masked_image_pub.publish(masked_msg)
            rospy.loginfo("Masked image published")


        except Exception as e:
            rospy.logerr("Failed to process image: %s", e)

if __name__ == '__main__':
    ip = ImageProcessor()
    rospy.spin()
