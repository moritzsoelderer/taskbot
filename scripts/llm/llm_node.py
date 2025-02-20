#!/usr/bin/env python3

import rospy
from dotenv import dotenv_values, find_dotenv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from llm_service import LLMService

env = dotenv_values(find_dotenv())

response_pub = rospy.Publisher(env["LLM_RESPONSES"], String, queue_size=1)

def image_callback(msg, service):
    rospy.loginfo("Image Message received")

    img = rospy.wait_for_message('/camera/image_raw', Image, timeout=2)
    answer = service.query(prompt=msg.data, image_msg=img)

    response = String()
    response.data = answer.choices[0].message.content

    response_pub.publish(response)


if __name__ == "__main__":
    print("Hallo LLM node!!!")
    rospy.init_node("llm_node")
    llm_service = LLMService(env["OPENAI_APIKEY"])
    print(env["LLM_PROMPTS"])

    rospy.Subscriber(env["LLM_PROMPTS"], String, lambda msg : image_callback(msg, llm_service), queue_size=1)
    rospy.spin()