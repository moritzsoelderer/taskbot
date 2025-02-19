#!/usr/bin/env python3

import rospy
from dotenv import dotenv_values, find_dotenv
from sensor_msgs.msg import Image
from llm_service import LLMService


def image_callback(msg, service):
    print("Image Message received")
    response = service.query(prompt="Describe the Image:", image_msg=msg)
    print(response)


if __name__ == "__main__":
    rospy.init_node("llm_node")

    env = dotenv_values(find_dotenv())
    llm_service = LLMService(env["OPENAI_APIKEY"])

    rospy.Subscriber('/camera/image_raw', Image, callback=lambda msg: image_callback(msg, llm_service))
    rospy.spin()