import rospy
import openai
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import base64
from io import BytesIO


class LLMService():

    def __init__(self, api_key: str):
        self.api_key = api_key
        self.client = openai.OpenAI(api_key=api_key)
        self.cv_bridge = CvBridge()


    def query(self, prompt: str, image_msg: Image) -> str:
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr("Error converting ROS Image message to OpenCV: %s", str(e))
            return

        # Convert the OpenCV image to base64
        img_b64_str = self.convert_image_to_base64(cv_image)

        # Send the image to OpenAI API
        response = self.query_base64(prompt, img_b64_str)
        return response


    def query_base64(self, prompt: str, img_b64_str: str):
        response = self.client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:image/jpeg;base64,{img_b64_str}"},
                        },
                    ],
                }
            ],
        )
        return response


    def convert_image_to_base64(self, cv_image):
        _, img_encoded = cv2.imencode('.jpg', cv_image)
        img_bytes = img_encoded.tobytes()

        img_b64_str = base64.b64encode(img_bytes).decode('utf-8')
        return img_b64_str
