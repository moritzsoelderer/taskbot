from pymycobot import MyCobot
from sensor_msgs.msg import JointState
from taskbot.msg import AprilTagDetectionArray
import rospy
import numpy as np
from time import sleep
from tf.transformations import euler_from_matrix
import math


class RobotControl():
    def __init__(self):
        self.speed = 10
        self.mc = MyCobot("/dev/ttyACM0", 115200)
        self.mc.set_fresh_mode(1)
        rospy.Rate(10)
        
        self.distance_threshold = (1.5, 5)
        self.TARGET_SIZE = 100 # 100 relates to ~ 2s
        self.DIFF_CAMERA_TO_ORIGIN_X = -(275+75)
        self.DIFF_CAMERA_TO_ORIGIN_Y = -1
        self.MOVEMENT_THRESHOLD_T = 5
        self.MOVEMENT_THRESHOLD_R = 5

        assert self.mc.get_coords() is not None

    def move_object(self, id_object, id_target):
        object_pose = self.find_pose(id_object)
        target_pose = self.find_pose(id_target)

        self.mc.set_gripper_state(0, self.speed)
        sleep(2)
        self.move_gripper_to_pose(object_pose, grasp=True)
        self.distance_threshold = (2, 5)
        self.move_gripper_to_pose(target_pose, grasp=False)

        rc.mc.send_angles([0,0,0,0,0,0], 10)

        return

    def find_pose(self, tag_id):
        rospy.loginfo(f"Finding pose for tag {tag_id}")
        pose_ts = []
        pose_rs = []

        def callback(msg):
            nonlocal pose_ts, pose_rs
            for detection in msg.detections:
                if detection.id == tag_id:
                    pose_ts.append(detection.pose_t)
                    pose_rs.append(detection.pose_R)

        sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback, queue_size=1)

        while len(pose_ts) < self.TARGET_SIZE: # ToDo: Add time condition
            rospy.sleep(0.05)
        
        print(len(pose_ts))
        sub.unregister()

        if not pose_ts:
            rospy.logwarn(f"No detections found for tag ID {tag_id}")
            raise

        avg_pose_t = np.mean(pose_ts, axis=0)
        avg_pose_r = np.mean(pose_rs, axis=0)
        
        return avg_pose_t, avg_pose_r

    def move_gripper_to_pose(self, pose, grasp=True):
        translation, rotation = pose
        point_x = self.DIFF_CAMERA_TO_ORIGIN_X - translation[1] * 1000
        point_y = self.DIFF_CAMERA_TO_ORIGIN_Y - translation[0] * 1000
        point = [point_x, point_y, 100, 90, -30, -90]

        parameter_idx = 0
        parameter_updates = [0, 0, 0, 0, +5, -10, +10, +5, -20, +25, -30, +35, -40, +45, -50]

        # while not distance_to_point < threshold:
        while not self.distance_less_then_threshold(point):
            current_target = point.copy()
            current_target_with_xy_adaption = point.copy()
            # while not distance_to_current_target < threshold:
            while not self.distance_less_then_threshold(current_target):
                # somehow record distance
                d0_t, d0_r = self.get_distance(current_target)
                # try to move to current target
                rospy.loginfo(f"Current position: {self.mc.get_coords()}")
                rospy.loginfo(f"Target point: {current_target}")
                rospy.loginfo(f"Try moving to coords: {current_target_with_xy_adaption}")
                self.mc.send_coords(current_target_with_xy_adaption, self.speed, 0)
                # sleep n sec
                sleep(7)
                rospy.loginfo(f"Position after move: {self.mc.get_coords()}")
                d1_t, d1_r = self.get_distance(current_target)
                print(d0_t, d1_t, d0_r, d1_r)
                if self.distance_less_then_threshold(current_target):
                    break
                # if closer:
                if d0_t - d1_t > self.MOVEMENT_THRESHOLD_T or d0_r - d1_r > self.MOVEMENT_THRESHOLD_R:
                    # repeat -> continue
                    continue
                # else (stuck):
                else:
                    # adapt parameters
                    if parameter_idx >= len(parameter_updates):
                        parameter_idx = 0

                    # adapt sideways value in direction of failure
                    distance_t, distance_r = self.get_distance(current_target)
                    if distance_t > self.distance_threshold[0]:
                        diff_x = current_target[0] - self.get_mc_coords()[0]
                        diff_y = current_target[1] - self.get_mc_coords()[1]
                        current_target_with_xy_adaption[0] += self.sign(diff_x)
                        current_target_with_xy_adaption[1] += self.sign(diff_y)

                    if abs(current_target_with_xy_adaption[0] - current_target[0]) > 20: raise


                        # if abs(current_target_with_xy_adaption[0] - current_target[0]) > 20: current_target_with_xy_adaption[0] = current_target[0]
                        # if abs(current_target_with_xy_adaption[1] - current_target[1]) > 20: current_target_with_xy_adaption[1] = current_target[1]
                    
                    # if distance_r > self.distance_threshold[1]:
                    current_target_with_xy_adaption[3] += 0.5 * parameter_updates[parameter_idx]
                    current_target_with_xy_adaption[4] += 0.5 * parameter_updates[parameter_idx]
                    current_target_with_xy_adaption[5] += 0.5 * parameter_updates[parameter_idx]
                    parameter_idx += 1

        point[2] -= 30
        self.mc.send_coords(point, self.speed, 0)
        sleep(3)

        rospy.loginfo(f"Final position: {self.mc.get_coords()}")

        grasp_target = int(grasp)
        self.mc.set_gripper_state(grasp_target, self.speed)
        sleep(1)
        point[2] += 20
        self.mc.send_coords(point, self.speed)
        sleep(2)

    def distance_less_then_threshold(self, point):
        threshold_t, threshold_r = self.distance_threshold
        diff_t, diff_r = self.get_distance(point)
        rospy.loginfo(f"Pose_t difference: {diff_t}, pose_r difference: {diff_r}")
        return diff_t < threshold_t and diff_r < threshold_r
    
    def get_distance(self, point):
        mc_coords = self.get_mc_coords()
        diff = np.absolute(np.array(point) - np.array(mc_coords))
        diff_t = sum(diff[:2])
        diff_r = sum(diff[3:])
        return diff_t, diff_r
    
    def get_mc_coords(self):
        mc_coords = None
        while mc_coords is None:
            mc_coords = self.mc.get_coords()
        return mc_coords
    
    def sign(self, num):
        return -1 if num < 0 else 1
