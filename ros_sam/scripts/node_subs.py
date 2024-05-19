#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Int32MultiArray
from ros_sam_msgs.srv import Segmentation, SegmentationRequest

class ImageSegmenter:
    def __init__(self):
        rospy.init_node('image_segmenter')

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.composite_pub = rospy.Publisher('/segmented/composite_image', Image, queue_size=10)
        self.segmentation_service = rospy.ServiceProxy('ros_sam/segment', Segmentation)

    def image_callback(self, img_msg):
        rospy.loginfo("Received an image, processing...")
        try:
            # Convert the ROS image message to a CV2 image
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

            # Prepare the segmentation request
            req = SegmentationRequest()
            req.image = img_msg
            req.query_points = [Point(x=100, y=100)]  # Example point, adjust as necessary
            req.query_labels = [1]  # Example label, adjust as necessary
            req.boxes = Int32MultiArray(data=[])  # No boxes, adjust as necessary
            req.multimask = True
            req.logits = False

            # Call the segmentation service
            response = self.segmentation_service(req)

            # Overlay masks on the original image
            for mask_msg in response.masks:
                mask = self.bridge.imgmsg_to_cv2(mask_msg, desired_encoding='8UC1')
                colored_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                colored_mask = (colored_mask * np.array([255, 200, 40])).astype(np.uint8)  # Color and alpha for the mask
                cv_image = cv2.addWeighted(cv_image, 1.0, colored_mask, 0.5, 0)

            # Convert the CV2 image back to ROS image message and publish
            composite_img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.composite_pub.publish(composite_img_msg)
            rospy.loginfo("Composite image published")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    segmenter = ImageSegmenter()
    rospy.spin()
