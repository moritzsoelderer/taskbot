import time
import rospy
from moveit_commander import MoveGroupCommander
from std_msgs.msg import String
from taskbot.msg import BoolOrNull
from dotenv import find_dotenv, dotenv_values
from grasp import GraspWithMoveIt

class ScenarioLogic:
    def __init__(self):
        rospy.init_node("scenario_logic")
        self.arm = MoveGroupCommander("arm_group")
        self.backoff = 2  # Initial backoff time
        self.backoff_increment = 5
        self.answer = "needed again"
        self.env = dotenv_values(find_dotenv())

        self.pub = rospy.Publisher(self.env["USER_QUESTIONS"], String, queue_size=1)
        rospy.Subscriber(self.env["USER_ANSWERS"], BoolOrNull, self.user_response_callback, queue_size=1)

        self.user_response = None
        self.last_query_time = None  # Timestamp for last user query
        self.last_usage_time = None  # Timestamp for last detected knife usage
        self.grasp = GraspWithMoveIt()

    def is_in_usage(self):
        #LLM to detect if the knife is in use
        # replace comment later
        in_use = False  # Assume not in use for now

        if in_use:
            self.last_usage_time = time.time()  # Update last usage timestamp
        
        return in_use

    def user_query(self):
        #Ask user if the knife is needed again
        current_time = time.time()

        # First query: Ask instantly
        if self.last_query_time is None:
            should_ask = True
        else:
            # Only ask if n second not in use
            if self.last_usage_time:
                time_since_last_use = current_time - self.last_usage_time
                should_ask = time_since_last_use >= 7
            else:
                should_ask = False

        if should_ask:
            question = String()
            question.data = "Do you still need this?"
            self.pub.publish(question)
            rospy.loginfo("Asked user: Do you still need this?")

            self.last_query_time = current_time  # Update last query timestamp
            self.user_response = None

            timeout = time.time() + 10  # Timeout after 10 seconds
            while self.user_response is None and time.time() < timeout:
                time.sleep(1)

            if self.user_response is None:
                rospy.logwarn("No response from user, assuming 'needed again'.")
                return "needed again"
            elif self.user_response:
                self.current_backoff += self.backoff_increment
                rospy.loginfo(f"User still needs the knife. Increasing backoff to {self.current_backoff} seconds.")
                return "needed again"
            else:
                return "not needed again"
        else:
            rospy.loginfo("Skipping user query due to recent knife usage.")
            return "needed again"

    def user_response_callback(self, msg):
        if msg.data == BoolOrNull.TRUE:
            self.user_response = True
        elif msg.data == BoolOrNull.FALSE:
            self.user_response = False
        else:
            self.user_response = None

    #Move object to target location
    def move_object(self, id_object, id_target):
        rospy.loginfo(f"Moving object {id_object} to target {id_target}.")
        self.grasp.run()
        rospy.loginfo("Move completed.")

    def run(self):
        rospy.loginfo("Starting scenario logic...")

        # Wait until the knife is in use
        while not self.is_in_usage():
            time.sleep(1)

        while self.answer == "needed again":
            # Wait while knife is in use
            while self.is_in_usage():
                time.sleep(1)

            # Start counter for inactivity detection
            start_time = time.time()
            while not self.is_in_usage():
                if time.time() - start_time >= self.backoff:
                    self.answer = self.user_query()
                    break
                time.sleep(1)

            if self.answer == "needed again":
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
    logic = ScenarioLogic()
    logic.run()
    rospy.spin()
