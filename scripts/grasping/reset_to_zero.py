#!/usr/bin/env python3
import rospy
import time
from pymycobot import MyCobot

def reset_to_zero(mc):
    # Gets the current angle of all joints
    angles = mc.get_angles()
    print(angles)
    mc.set_color(73, 226, 73)

    for joint_id in range(1, 7):
        mc.send_angle(joint_id, 0, 50)
        time.sleep(1)
    angles = mc.get_angles()
    print(angles)

if __name__ == "__main__":
    rospy.init_node("mycobot_test", anonymous=True)
    port = rospy.get_param("~port", "/dev/ttyACM0")
    baud = rospy.get_param("~baud", 115200)
    print(port, baud)
    mc = MyCobot(port, baud)

    reset_to_zero(mc)
