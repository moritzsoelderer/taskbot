#!/usr/bin/env python3
import rospy
import time
from pymycobot import MyCobot

def test():
    rospy.init_node("mycobot_test", anonymous=True)
    port = rospy.get_param("~port", "/dev/ttyACM0")
    baud = rospy.get_param("~baud", 115200)
    print(port, baud)
    mc = MyCobot(port, baud)

    # Gets the current angle of all joints
    angles = mc.get_angles()
    print(angles)

    while True:
        mc.send_angle(3, (angles[2] + 20 ) % 90, 20)
        time.sleep(1)
        angles = mc.get_angles()
        print(angles)
    rospy.spin()

if __name__ == "__main__":
    test()
