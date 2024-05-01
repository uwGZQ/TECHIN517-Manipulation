import rospy
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
from cv_bridge import CvBridge
import cv2

def callback(data, camera_info_msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    cam_model = PinholeCameraModel()
    cam_model.fromCameraInfo(camera_info_msg)

    u, v = 640, 80
    depth = cv_image[v, u]

    (x, y, z) = cam_model.projectPixelTo3dRay((u, v))
    x *= depth
    y *= depth
    z = depth

    print(f"3D Coordinates in Camera Frame: x={x}, y={y}, z={z}")

def listener():
    rospy.init_node('realsense_listener', anonymous=True)
    camera_info_msg = rospy.wait_for_message("/camera/aligned_depth_to_color/camera_info", CameraInfo)
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, callback, camera_info_msg)
    rospy.spin()

if __name__ == '__main__':
    listener()
