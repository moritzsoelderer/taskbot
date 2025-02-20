#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from taskbot.msg import BoolOrNull
import time
from dotenv import find_dotenv, dotenv_values


env = dotenv_values(find_dotenv())   # take environment variables from .env.

pub = rospy.Publisher(env["USER_QUESTIONS"], String, queue_size=1)

if __name__ == "__main__":
    rospy.init_node(name="demo_user_interaction_node")
    rospy.Subscriber(env["USER_ANSWERS"], BoolOrNull, lambda k: print("Answer:", k.data), queue_size=1)
    rospy.loginfo("Done subscribing")

    rospy.loginfo("Asking...")
    question = String()
    question.data = "Do you still need this?"
    pub.publish(question)
    rospy.loginfo("Done Asking")

    time.sleep(5)
    rospy.spin()