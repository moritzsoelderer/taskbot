#!/usr/bin/env python3

import rospy
import time
import sys
import math
import numpy as np

from pymycobot import MyCobot
import moveit_commander
from geometry_msgs.msg import PoseStamped
from taskbot.msg import AprilTagDetectionArray, Move
from grasp_moveit_service import GraspMoveItService
from tf.transformations import euler_from_matrix
from dotenv import find_dotenv, dotenv_values
from grasp_moveit_helper import determine_pos_orient, add_plane


def move_to_apriltag(msg, robot_service):
    for detection in msg.detections:
        print("April Tag Id:", detection.id)
        dist = [0, 0, 0]
        pos = detection.pose_t
        orient = np.array(detection.pose_R).reshape(3, 3)
        orient_euler = euler_from_matrix(orient)

        print("pos", pos)
        print("orient", orient)
        print("orient_euler", orient_euler)

        pos_new, orient_new = determine_pos_orient(pos, orient, dist)
        orient_new_euler = euler_from_matrix(orient_new)
        print("pos_new", pos_new)
        print("orient_new", orient_new)
        print("orient_new_euler", orient_new_euler)

        pos_new_corrected = pos_new + np.array([0, 0, 0.1]) # account for gripper

        robot_service.move_facing_downwards(pos_new, orient_new_euler[2])
        rospy.sleep(10)


env = dotenv_values(find_dotenv())

pub = rospy.Publisher(env["ROBOT_MOVES"], Move, queue_size=1)

if __name__ == "__main__":
    print("hallo")
    rospy.init_node("grasp_ms_node", anonymous=True)

    move = Move()
    move.object_id = 1
    move.target_id = 0
    pub.publish(move)

    # Init MoveIt & Scene
    #moveit_commander.roscpp_initialize(sys.argv)
    #arm = moveit_commander.MoveGroupCommander("arm_group")
    #scene = moveit_commander.PlanningSceneInterface()
    #add_plane(scene)

    # Init Cobot
    #baud = rospy.get_param("~baud", 115200)
    #port = rospy.get_param("~port", "/dev/ttyACM0")
    #mc = MyCobot(port, baud)

    #robot_service = GraspMoveItService(scene, arm, mc)


    # move
    #robot_service.reset_to_initial_position()
    #robot_service.open_gripper()
    #robot_service.move_facing_downwards([-0.12, 0.0, 0.065],  -math.pi/4)
    #[ 0.00287037 -0.2031672   0.56708336]
    # -0.12368217  0.00218314 -0.06080169
    #robot_service.move_facing_downwards([-0.12, -0.14, 0.065],  -math.pi/4)
    #robot_service.close_gripper()
    #robot_service.move_facing_downwards([0, 0.2, 0.15], 0)
    #robot_service.open_gripper()
    #robot_service.move_facing_downwards([0.15, 0.15, 0.25], math.pi/2)

    # move to april tag
    #sub = rospy.Subscriber("tag_detections", AprilTagDetectionArray, callback=lambda msg: move_to_apriltag(msg, robot_service), queue_size=1)
    #rospy.sleep(1)
    #sub.unregister()
    rospy.spin()