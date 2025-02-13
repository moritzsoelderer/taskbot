#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

class CameraPublisher:
    def __init__(self):
        # Initialize the node
        rospy.init_node('camera_publisher', anonymous=True)

        # Create publishers
        self.image_pub = rospy.Publisher('camera/image_raw', Image, queue_size=10)
        self.camera_info_pub = rospy.Publisher('camera/camera_info', CameraInfo, queue_size=10)

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Initialize the camera
        self.camera = cv2.VideoCapture(0)  # Use 0 for default camera

        # Set camera properties (optional)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # Camera calibration parameters
        self.K = np.array([
            [503.52212697, 0.0, 319.5], # fx, 0, cx
            [0.0, 501.75233068, 239.5], # 0, fy, cy
            [0.0, 0.0, 1.0]
        ])

        # Initialize CameraInfo message
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.height = 480
        self.camera_info_msg.width = 640
        self.camera_info_msg.K = self.K.flatten().tolist()
        self.camera_info_msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion
        # Set identity matrix for rectification
        self.camera_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        # Set projection matrix (just K with an extra column for tx, ty, tz)
        self.camera_info_msg.P = [
            self.K[0,0], self.K[0,1], self.K[0,2], 0,
            self.K[1,0], self.K[1,1], self.K[1,2], 0,
            self.K[2,0], self.K[2,1], self.K[2,2], 0
        ]

        # Check if camera opened successfully
        if not self.camera.isOpened():
            rospy.logerr("Error opening camera")
            return

        rospy.loginfo("Camera Publisher Node Started")

    def publish_frames(self):
        rate = rospy.Rate(30)  # 30Hz

        while not rospy.is_shutdown():
            # Capture frame
            ret, frame = self.camera.read()

            if ret:
                try:
                    # Get current timestamp
                    now = rospy.Time.now()

                    # Convert OpenCV image to ROS message
                    ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    ros_image.header.stamp = now

                    # Update camera info timestamp
                    self.camera_info_msg.header.stamp = now

                    # Publish the image and camera info
                    self.image_pub.publish(ros_image)
                    self.camera_info_pub.publish(self.camera_info_msg)

                except Exception as e:
                    rospy.logerr(f"Error converting/publishing image: {str(e)}")

            rate.sleep()

    def cleanup(self):
        self.camera.release()
        cv2.destroyAllWindows()
        rospy.loginfo("Camera Publisher Node Stopped")

if __name__ == '__main__':
    try:
        camera_pub = CameraPublisher()
        camera_pub.publish_frames()
    except rospy.ROSInterruptException:
        pass
    finally:
        camera_pub.cleanup()
