import time
import rospy
from moveit_commander import MoveGroupCommander
from grasp import GraspWithMoveIt
from llm.llm_service import LLMService
from dotenv import find_dotenv, dotenv_values

from std_msgs.msg import String
from taskbot.msg import BoolOrNull


class ScenarioLogic:

    def __init__(self):
        self.env = dotenv_values(find_dotenv())

        self.arm = MoveGroupCommander("arm_group")
        self.grasp = GraspWithMoveIt()
        self.question_pub = rospy.Publisher(env["USER_QUESTIONS", String])
        self.prompt_pub = rospy.Publisher("llm_prompt", String)

        self.last_query_time = None  # Timestamp for last user query
        self.last_usage_time = None  # Timestamp for last detected knife usage
        self.backoff = 2  # Initial backoff time
        self.backoff_increment = 5


    def is_in_usage(self):
        prompt = String()
        prompt.data = self.env["PROMPT"]

        self.prompt_pub.publish(prompt)
        msg = rospy.wait_for_message("llm_response", String, timeout=10)

        response = msg.data.lower()
        in_use = True if response.contains("yes") else False

        if in_use:
            self.last_usage_time = time.time()  # Update last usage timestamp
        
        return in_use


    def user_query(self):
        #Ask user if the knife is needed again
        current_time = time.time()

        # First query: Ask instantly
        if self.last_query_time is None:
            should_ask = True
        # Only ask if n second not in use
        elif self.last_usage_time:
            time_since_last_use = current_time - self.last_usage_time
            if time_since_last_use >= 7:
                return self.ask_user()
        else:
            rospy.loginfo("Skipping user query due to recent knife usage.")
            return "needed again"


    def ask_user(self):
        question = String()
        question.data = "Do you still need this?"

        self.question_pub.publish(question)
        msg = rospy.wait_for_message(self.env["USER_ANSWERS"], BoolOrNull, timeout=10)
        response = msg.data

        rospy.loginfo(f"Asked user: {question}")

        self.last_query_time = current_time  # Update last query timestamp

        if user_response is None:
            rospy.logwarn("No response from user, assuming 'needed again'.")
            return "needed again"
        elif user_response:
            self.current_backoff += self.backoff_increment
            rospy.loginfo(f"User still needs the knife. Increasing backoff to {self.current_backoff} seconds.")
            return "needed again"
        else:
            return "not needed again"


    def move_object(self, id_object, id_target):
        rospy.loginfo(f"Moving object {id_object} to target {id_target}.")
        self.grasp.run()
        rospy.loginfo("Move completed.")


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
        self.move_object(id_object=1, id_target=2)  # Correct apriltag IDs?
        rospy.loginfo("Scenario complete.")

if __name__ == "__main__":
    rospy.init_node("scenario_logic")
    logic = ScenarioLogic()
    logic.run()
    rospy.spin()
