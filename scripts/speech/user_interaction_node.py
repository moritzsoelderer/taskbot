#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from taskbot.msg import BoolOrNull
from mapper import boolToBoolOrNull
from user_interaction_service import UserLanguageInteractionService


service = UserLanguageInteractionService("de-DE")
pub = rospy.Publisher("user_answers", BoolOrNull, queue_size=1)


def askAndPublish(pub, question: String):
    print("Question:", question.data)
    print("Asking...")
    ans = service.askUserJaNein(question.data)

    answer = BoolOrNull()
    answer.data = boolToBoolOrNull(ans)
    
    pub.publish(answer)
    print("Done Asking")


if __name__ == "__main__":
    rospy.init_node(name="user_interaction_node")

    rospy.Subscriber("user_questions", String, lambda msg: askAndPublish(pub, msg), queue_size=1)
    print("Done subscribing")
    
    rospy.spin()
