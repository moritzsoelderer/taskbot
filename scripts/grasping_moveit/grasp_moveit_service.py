#!/usr/bin/env python3

import rospy
import time
import sys
import math
import numpy as np
from pymycobot import MyCobot
from sensor_msgs.msg import JointState
from taskbot.msg import AprilTagDetectionArray
from grasp_moveit_helper import create_pose_stamped, create_alt


DOWNWARDS = np.array([
    math.radians(-92),
    math.radians(-146),
    0
])

DOWNWARDS_ALT = np.array([
    math.radians(88),
    math.radians(-36),
    0
])

DOWNWARDS_QUAT = {
    "x": -0.22024299643998094,
    "y": -0.6275296776578194,
    "z": -0.7013115338314049,
    "w": 0.2566352639308447
}


class GraspMoveItService:
    def __init__(self, scene, arm, mc):
        self.arm = arm
        self.scene = scene
        self.mc = mc

        self.reference_frame = "joint1"
        self.arm.set_planner_id("RRTstar")

        self.end_effector_link = self.arm.get_end_effector_link()
        self.arm.set_pose_reference_frame(self.reference_frame)

        self.arm.allow_replanning(True)
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.005)
        self.arm.set_planning_time(10)

        self.DIFF_CAMERA_TO_ORIGIN_X = -(0.275+0.075)
        self.DIFF_CAMERA_TO_ORIGIN_Y = -0.001
        self.DIFF_VECTOR = np.array([-0.325, -0.001, 0.0])
        self.TARGET_SIZE = 30 # 100 relates to ~ 2s


    def open_gripper(self):
        self.mc.set_gripper_state(0, 80)


    def close_gripper(self):
        self.mc.set_gripper_state(1, 80)


    def reset_to_initial_position(self):
        rospy.loginfo(f"Setting angles to zero...")
        rospy.loginfo(f"Setting angles {self.mc.get_angles()} to zero...")

        for joint_id in range(1, 7):
            #self.mc.send_angle(joint_id, 0, 40)
            time.sleep(1)

        rospy.loginfo(f"Set angles to {self.mc.get_angles()}.")


    def move_object(self, object_id: int, target_id: int):
        object_pose, object_orient = self.find_pose(object_id)
        target_pose, target_orient = self.find_pose(target_id)

        #object_pose = [
            #self.DIFF_CAMERA_TO_ORIGIN_X - object_pose[0],
            #self.DIFF_CAMERA_TO_ORIGIN_Y - object_pose[1],
            #0.065
        #]
        #target_pose = [
            #self.DIFF_CAMERA_TO_ORIGIN_X - target_pose[0],
            #self.DIFF_CAMERA_TO_ORIGIN_Y - target_pose[1],
            #0.08
        #]
        #object_orient = 

        # determine coordinates in cobot vector space
        object_pose, object_orient = determine_pos_orient(object_pose, object_orient, self.DIFF_VECTOR)
        target_pose, target_orient = determine_pos_orient(target_pose, target_orient, self.DIFF_VECTOR)
        rospy.loginfo(f"Object Pos.: {object_pose} | Object Orient. (Euler): {object_orient}")
        rospy.loginfo(f"Object Pos.: {target_pose} | Object Orient. (Euler): {target_orient}")

        self.open_gripper()
        self.move_facing_downwards(object_pose, 0)
        self.close_gripper()
        rospy.logwarn("DEBUG")
        self.move_facing_downwards(target_pose, 0)
        self.open_gripper


    def move_facing_downwards(self, pos, z_orient, timeout=2):
        rospy.loginfo(f"Position: {pos} | Z-Orientation: {z_orient}")
     
        pose = create_pose_stamped(self.reference_frame, rospy.Time.now(), pos, DOWNWARDS + [0, 0, z_orient])
        pose_alt = create_alt(pose, DOWNWARDS_ALT + [0, 0, z_orient])

        if z_orient < 0:
            self.plan_or_else(pose_alt, pose)
        else:
            self.plan_or_else(pose, pose_alt)
        time.sleep(timeout)


    def find_pose(self, tag_id):
        rospy.loginfo(f"Finding pose for tag {tag_id}...")
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
            rospy.sleep(0.01)
        
        rospy.loginfo(f"Took {len(pose_ts)} samples for tag {tag_id}")
        sub.unregister()

        if not pose_ts:
            rospy.logwarn(f"No detections found for tag ID {tag_id}")
            raise

        avg_pose_t = np.mean(pose_ts, axis=0)
        avg_pose_r = np.mean(pose_rs, axis=0)
        
        rospy.loginfo(f"Avg. Position: {avg_pose_t} | Avg. Orientation (Euler) {euler_from_matrix(avg_pose_r)}")
        return avg_pose_t, avg_pose_r


    def plan_or_else(self, if_pose, else_pose):
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(if_pose, self.end_effector_link)

        plan = self.arm.plan()
        if plan[0]:  # Plan success
            rospy.loginfo("Executing first motion plan...")
            self.arm.execute(plan[1], wait=True)  # This will fully block
            self.arm.stop()
            self.arm.clear_pose_targets()
            rospy.loginfo("Motion execution completed.")
        else:
            rospy.logwarn("Motion planning failed - Trying second Pose")
            self.arm.set_start_state_to_current_state()
            self.arm.set_pose_target(else_pose, self.end_effector_link)
            plan = self.arm.plan()
            if plan[0]:  # Plan success
                rospy.loginfo("Executing alternative motion plan...")
                self.arm.execute(plan[1], wait=True)  # This will fully block
                self.arm.stop()
                self.arm.clear_pose_targets()
                rospy.loginfo("Motion execution completed.")


    #def add_constraint(self):
     #   position_constraint = PositionConstraint()
      #  position_constraint.link_name = self.end_effector_link  # Replace with your end-effector link
       # position_constraint.header.frame_id = "joint1"  # Reference frame for the constraint

        # Set position bounds for X and Y (free motion), but constrain Z to the highest possible value
        #position_constraint.constraint_region.position.x = 0.5  # Allow motion within this X range
        #position_constraint.constraint_region.position.y = 0.5  # Allow motion within this Y range
        #position_constraint.constraint_region.position.z = 0.1
        #self.arm.set_path_constraints(position_constraint)

