#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from taskbot.msg import BoolOrNull
from mapper import boolToBoolOrNull
from user_interaction_service import UserLanguageInteractionService
from dotenv import find_dotenv, dotenv_values


env = dotenv_values(find_dotenv())  # take environment variables from .env.

service = UserLanguageInteractionService("en-US")
pub = rospy.Publisher(env["USER_ANSWERS"], BoolOrNull, queue_size=1)


def askAndPublish(pub, question: String):
    ans = service.askUserBinary(question.data, ["yes", "yeah"], ["no", "nah"])

    answer = BoolOrNull()
    answer.data = boolToBoolOrNull(ans)
    
    pub.publish(answer)


if __name__ == "__main__":
    rospy.init_node(name="user_interaction_node")
    rospy.loginfo(f"user_interaction_node started.")

    rospy.Subscriber(env["USER_QUESTIONS"], String, lambda msg: askAndPublish(pub, msg), queue_size=1)
    
    rospy.spin()
