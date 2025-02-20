#!/usr/bin/env python3

import rospy
from dotenv import dotenv_values, find_dotenv
from grasp_moveit_service import GraspMoveItService
from taskbot.msg import Move
from std_msgs.msg import Bool
import moveit_commander
from geometry_msgs.msg import PoseStamped
import sys
from pymycobot import MyCobot
from grasp_moveit_helper import init_mycobot, add_plane


env = dotenv_values(find_dotenv())
pub = rospy.Publisher(env["ROBOT_MOVE_RESPONSES"], Bool, queue_size=1)

def move_robot(move: Move, robot_service: GraspMoveItService) -> None:
    robot_service.reset_to_initial_position()
    robot_service.move_object(move.object_id, move.target_id)

    boolean = Bool()
    boolean.data = True
    pub.publish(boolean)


if __name__ == "__main__":
    rospy.init_node('robot_control', anonymous=True)
    rospy.loginfo("robot_control node started.")

    # Init MoveIt, Scene & Cobot
    moveit_commander.roscpp_initialize(sys.argv)
    arm = moveit_commander.MoveGroupCommander("arm_group")
    scene = moveit_commander.PlanningSceneInterface()
    add_plane(scene)
    mc = init_mycobot()

    # Init Service and Subscription
    robot_service = GraspMoveItService(scene, arm, mc)
    rospy.Subscriber(
        env["ROBOT_MOVES"],
        Move,
        callback=lambda msg: move_robot(msg, robot_service),
        queue_size=1
    )

    rospy.spin()