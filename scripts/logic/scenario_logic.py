#!/usr/bin/env python3

import time
import rospy
import pygame
from moveit_commander import MoveGroupCommander
from dotenv import find_dotenv, dotenv_values

from std_msgs.msg import String, Bool
from taskbot.msg import BoolOrNull, Move

env = dotenv_values(find_dotenv())
question_pub = rospy.Publisher(env["USER_QUESTIONS"], String, queue_size=1)
prompt_pub = rospy.Publisher(env["LLM_PROMPTS"], String, queue_size=1)
move_pub = rospy.Publisher(env["ROBOT_MOVES"], Move, queue_size=1)

class ScenarioLogic:

    def __init__(self):
        self.env = env

        self.question_pub = question_pub
        self.prompt_pub = prompt_pub
        self.move_pub = move_pub

        self.last_query_time = None  # Timestamp for last user query
        self.last_usage_time = None  # Timestamp for last detected knife usage
        self.backoff = 2  # Initial backoff time
        self.current_backoff = self.backoff
        self.backoff_increment = 5


    def is_in_usage(self):
        prompt = String()
        prompt.data = self.env["LLM_PROMPT"]

        self.prompt_pub.publish(prompt)
        msg = rospy.wait_for_message(self.env["LLM_RESPONSES"], String, timeout=10)

        response = msg.data.lower()
        in_use = True if "yes" in response else False

        if in_use:
            self.last_usage_time = time.time()  # Update last usage timestamp
        
        return in_use


    def user_query(self):
        #Ask user if the knife is needed again
        current_time = time.time()
        rospy.loginfo("Querying user")

        # First query: Ask instantly
        if self.last_query_time is None:
            return self.ask_user(current_time)
        # Only ask if n second not in use
        elif self.last_usage_time:
            time_since_last_use = current_time - self.last_usage_time
            if time_since_last_use >= 7:
                return self.ask_user(current_time)
        else:
            rospy.loginfo("Skipping user query due to recent knife usage.")
            return "needed again"


    def ask_user(self, current_time):
        question = String()
        question.data = "Do you still need this?"

        self.question_pub.publish(question)
        msg = rospy.wait_for_message(self.env["USER_ANSWERS"], BoolOrNull, timeout=25)
        rospy.logwarn(f"User Message: {msg}")
        user_response = self.boolOrNullToBool(msg.data)

        rospy.loginfo(f"Asked user: {question.data}")

        self.last_query_time = current_time  # Update last query timestamp

        if user_response is None:
            rospy.logwarn("No response from user, assuming 'needed again'.")
            return "needed again"
        elif user_response:
            self.current_backoff += self.backoff_increment
            rospy.loginfo(f"User said {user_response}")
            rospy.loginfo(f"User still needs the knife. Increasing backoff to {self.current_backoff} seconds.")
            return "needed again"
        else:
            return "not needed again"


    def boolOrNullToBool(self, boolOrNull: BoolOrNull) -> bool:
        if boolOrNull == BoolOrNull.TRUE:
            return True
        elif boolOrNull == BoolOrNull.FALSE:
            return False
        else:
            return None


    def move_object(self, object_id, target_id):
        rospy.loginfo(f"Moving object {object_id} to target {target_id}.")
        move = Move()
        move.object_id = object_id
        move.target_id = target_id
        self.move_pub.publish(move)
        rospy.wait_for_message(self.env["ROBOT_MOVE_RESPONSES"], Bool, timeout=120)
        rospy.loginfo("Move completed.")


    def play_startup_sound(self):
        pygame.mixer.music.load(rospy.get_param("soft_startup_audio"))
        pygame.mixer.music.play()

        while pygame.mixer.music.get_busy():
            continue
        rospy.sleep(1)


    def run(self):
        rospy.loginfo("Starting scenario logic...")
        utensil_state = "needed again"

        # Wait until the knife is in use
        while not self.is_in_usage():
            time.sleep(1)

        while utensil_state == "needed again":
            # Wait while knife is in use
            while self.is_in_usage():
                time.sleep(1)

            # Start counter for inactivity detection
            rospy.loginfo("Started Inactivity Counter.")
            start_time = time.time()
            while not self.is_in_usage():
                if time.time() - start_time >= self.backoff:
                    utensil_state = self.user_query()
                    break
                time.sleep(1)

            if utensil_state == "needed again":
                while self.is_in_usage():
                    time.sleep(1)
                continue
            else:
                break

        # Clean up knife
        rospy.loginfo("Cleaning up knife...")
        self.move_object(object_id=1, target_id=0)
        rospy.loginfo("Scenario complete.")

if __name__ == "__main__":
    rospy.init_node("scenario_logic_node")
    rospy.loginfo("scenario_logic_node started.")
    logic = ScenarioLogic()
    pygame.mixer.init()

    rospy.sleep(4) # let other nodes fully initialize first
    logic.play_startup_sound()

    logic.run()
    rospy.spin()
