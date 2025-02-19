#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from taskbot.msg import AprilTagDetection, AprilTagDetectionArray
from dt_apriltags import Detector


class AprilTagDetectorNode:
    def __init__(self):
        rospy.init_node('apriltag_detector', anonymous=True)

        # Parameters
        self.tag_family = rospy.get_param('~tag_family', 'tag36h11')
        self.tag_size = rospy.get_param('~tag_size', 0.015)  # meters

        # Initialize detector
        self.detector = Detector(families=self.tag_family,
                               nthreads=1,
                               quad_decimate=1.0,
                               quad_sigma=0.0,
                               refine_edges=1,
                               decode_sharpening=0.25,
                               debug=0)

        self.bridge = CvBridge()

        # Camera parameters (will be updated when we receive CameraInfo)
        self.camera_params = None

        # Publishers and Subscribers
        self.tag_pub = rospy.Publisher('tag_detections',
                                     AprilTagDetectionArray,
                                     queue_size=10)

        self.image_sub = rospy.Subscriber('camera/image_raw',
                                         Image,
                                         self.image_callback,
                                         queue_size=1)

        self.camera_info_sub = rospy.Subscriber('camera/camera_info',
                                               CameraInfo,
                                               self.camera_info_callback,
                                               queue_size=1)

        rospy.loginfo("AprilTag Detector Node Started")

    def camera_info_callback(self, msg):
        if self.camera_params is None:
            fx = msg.K[0]
            fy = msg.K[4]
            cx = msg.K[2]
            cy = msg.K[5]
            self.camera_params = (fx, fy, cx, cy)
            rospy.loginfo("Camera parameters updated")

    def image_callback(self, msg):
        if self.camera_params is None:
            return

        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

            # Detect AprilTags
            tags = self.detector.detect(cv_image,
                                     True,
                                     self.camera_params,
                                     self.tag_size)

            # Create detection array message
            detection_array_msg = AprilTagDetectionArray()
            detection_array_msg.header = msg.header

            # Process each detection
            for tag in tags:
                detection = AprilTagDetection()
                detection.family = self.tag_family
                detection.id = tag.tag_id
                detection.hamming = tag.hamming
                detection.decision_margin = tag.decision_margin
                detection.pose_R = tag.pose_R.flatten().tolist()
                detection.pose_t = tag.pose_t.flatten().tolist()
                detection.pose_err = tag.pose_err
                detection.center = tag.center.tolist()
                # Flatten corners array and convert to list
                detection.corners = tag.corners.flatten().tolist()

                detection_array_msg.detections.append(detection)

            # Publish detections
            self.tag_pub.publish(detection_array_msg)

        except Exception as e:
            rospy.logerr(f"Error processing image: {str(e)}")

if __name__ == '__main__':
    try:
        detector_node = AprilTagDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
