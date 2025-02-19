#!/usr/bin/env python3

import rospy
from dotenv import dotenv_values, find_dotenv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from llm_service import LLMService

response_pub = rospy.Publisher('llm_response', String)

def image_callback(msg, service):
    print("Image Message received")

    img = rospy.wait_for_message('/camera/image_raw', Image, timeout=2)
    answer = service.query(prompt=msg.data, image_msg=img)

    response = String()
    response.data = answer

    response_pub.publish(response)

    print(response)


if __name__ == "__main__":
    rospy.init_node("llm_node")

    env = dotenv_values(find_dotenv())
    llm_service = LLMService(env["OPENAI_APIKEY"])

    rospy.Subscriber('llm_prompt', String, callback=lambda msg: image_callback(msg, llm_service))
    rospy.spin()