#!/usr/bin/env python3

import rospy
from dotenv import dotenv_values, find_dotenv
from grasping_service import RobotControl
from taskbot.msg import Move
from std_msgs.msg import Bool

env = dotenv_values(find_dotenv())

pub = rospy.Publisher(env["ROBOT_MOVE_RESPONSES"], Bool, queue_size=1)

def move_robot(move: Move, robot_control: RobotControl) -> None:
    rospy.loginfo("Received move command")
    robot_control.mc.send_angles([0,0,0,0,0,0], 10)

    robot_control.move_object(move.object_id, move.target_id)

    boolean = Bool()
    boolean.data = True
    pub.publish(boolean)



if __name__ == "__main__":
    rospy.init_node('robot_control', anonymous=True)
    robot_control = RobotControl()
    rospy.Subscriber(
        env["ROBOT_MOVES"],
        Move,
        callback=lambda msg: move_robot(msg, robot_control),
        queue_size=1
    )
    rospy.spin()