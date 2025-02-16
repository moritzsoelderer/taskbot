#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from taskbot.msg import BoolOrNull
import time

pub = rospy.Publisher("user_questions", String, queue_size=1)

if __name__ == "__main__":
    rospy.init_node(name="demo_user_interaction_node")
    rospy.Subscriber("user_answers", BoolOrNull, lambda k: print("Answer:", k.data), queue_size=1)
    print("Done subscribing")

    print("Asking...")
    question = String()
    question.data = "Hallo!"
    pub.publish(question)
    print("Done Asking")

    time.sleep(5)
    rospy.spin()