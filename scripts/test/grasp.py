#!/usr/bin/env python3
import rospy
import time
import sys
from pymycobot import MyCobot
import moveit_commander
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

class GraspWithMoveIt:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("grasp_with_moveit", anonymous=True)
        self.arm = moveit_commander.MoveGroupCommander("arm_group")
        self.scene = moveit_commander.PlanningSceneInterface()
        self.end_effector_link = self.arm.get_end_effector_link()

        self.reference_frame = "base_link"
        self.arm.set_pose_reference_frame(self.reference_frame)

        self.arm.allow_replanning(True)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.05)
        #self.arm.set_planning_time(10)

    def open_gripper(self, mc):
        mc.set_gripper_state(0, 80)
        time.sleep(9)

    def close_gripper(self, mc):
        mc.set_gripper_state(1, 80)
        time.sleep(9)

    def reset_to_initial_position(self, mc):
        # Move to zero angles
        angles = mc.get_angles()
        print(angles)
        for joint_id in range(1, 7):
            mc.send_angle(joint_id, 0, 40)
            time.sleep(1)
        angles = mc.get_angles()
        print(angles)

    # Set the target pose
    def move_to_pose(self, target_pose):
        self.arm.set_start_state_to_current_state()
        self.arm.set_planning_time(10.0)
        self.arm.set_pose_target(target_pose, self.end_effector_link)

        plan = self.arm.plan()
        if plan[0]:  # Plan success
            rospy.loginfo("Executing planned motion...")
            self.arm.execute(plan[1], wait=True)  # This will fully block
            self.arm.stop()
            self.arm.clear_pose_targets()
            rospy.loginfo("Motion execution completed.")
        else:
            rospy.logwarn("Motion planning failed!")


    def grasp(self, mc):
        self.reset_to_initial_position(mc)



        # Define intermediate number 0 Cartesian position (--> where to pick up object)
        intermediate0_position = PoseStamped()
        intermediate0_position.header.frame_id = self.reference_frame
        intermediate0_position.header.stamp = rospy.Time.now()
        intermediate0_position.pose.position.x = -0.25
        intermediate0_position.pose.position.y = 0.0
        intermediate0_position.pose.position.z = 0.15
        quat = quaternion_from_euler(-1.5708, -2.36, 0)
        intermediate0_position.pose.orientation.x = quat[0]
        intermediate0_position.pose.orientation.y = quat[1]
        intermediate0_position.pose.orientation.z = quat[2]
        intermediate0_position.pose.orientation.w = quat[3]

        # Define intermediate Cartesian position (--> where to pick up object)
        intermediate_position = PoseStamped()
        intermediate_position.header.frame_id = self.reference_frame
        intermediate_position.header.stamp = rospy.Time.now()
        intermediate_position.pose.position.x = -0.25
        intermediate_position.pose.position.y = 0.0
        intermediate_position.pose.position.z = 0.075
        quat = quaternion_from_euler(-1.5708, -2.36, 0)
        intermediate_position.pose.orientation.x = quat[0]
        intermediate_position.pose.orientation.y = quat[1]
        intermediate_position.pose.orientation.z = quat[2]
        intermediate_position.pose.orientation.w = quat[3]



        # Define final Cartesian position (--> where to put object)
        final_position = PoseStamped()
        final_position.header.frame_id = self.reference_frame
        final_position.header.stamp = rospy.Time.now()
        final_position.pose.position.x = 0.25
        final_position.pose.position.y = 0.0
        final_position.pose.position.z = 0.075
        quat = quaternion_from_euler(-1.5708, -2.36, 0)
        final_position.pose.orientation.x = quat[0]
        final_position.pose.orientation.y = quat[1]
        final_position.pose.orientation.z = quat[2]
        final_position.pose.orientation.w = quat[3]


        self.open_gripper(mc)
        self.move_to_pose(intermediate0_position)

        # Move robot to intermediate position
        self.move_to_pose(intermediate_position)

        self.close_gripper(mc)
        time.sleep(4)


        self.move_to_pose(final_position)
        rospy.loginfo("Reached final position, opening gripper now.")
        self.open_gripper(mc)



    def run(self):
        port = rospy.get_param("~port", "/dev/ttyACM0")
        baud = rospy.get_param("~baud", 115200)
        mc = MyCobot(port, baud)
        self.grasp(mc)
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    grasp_demo = GraspWithMoveIt()
    grasp_demo.run()
    rospy.spin()
