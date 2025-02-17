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
        # self.K = np.array([
        #     [484.09, 0.0, 313.45], # fx, 0, cx
        #     [0.0, 486.88, 244.77], # 0, fy, cy
        #     [0.0, 0.0, 1.0]
        # ])

        # Initialize CameraInfo message
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.height = 480
        self.camera_info_msg.width = 640
        self.camera_info_msg.K = [484.09392206416794, 0.0, 313.44707628106426, 0.0, 486.88216799355365, 244.7723326921671, 0.0, 0.0, 1.0]
        self.camera_info_msg.D = [0.20556447733404937, -0.24417131503030026, 0.0008799080148198596, -0.0006182501743163579, 0.0]  # No distortion
        # Set identity matrix for rectification
        self.camera_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        # Set projection matrix (just K with an extra column for tx, ty, tz)
        self.camera_info_msg.P = [504.35772705078125, 0.0, 312.560561603721, 0.0, 0.0, 507.0992431640625, 244.65665349823394, 0.0, 0.0, 0.0, 1.0, 0.0]

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
