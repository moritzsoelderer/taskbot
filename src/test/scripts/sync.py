#!/usr/bin/env python3

# McCobot
from pymycobot import MyCobot
from sensor_msgs.msg import JointState
import rospy
import numpy as np

i = 0
speed = 100
last_data = None

def callback(data):
    global last_data
    data_list = []

    for _, value in enumerate(data.position):
        data_list.append(round(value, 3))
    if last_data ==  data_list[:7]:
        return
    else:
        last_data = data_list[:7]

    mc.send_angles(np.rad2deg(data_list[:6]).tolist(), speed)
    gripper_value = int(abs(-0.7 - data_list[6]) * 117)
    mc.set_gripper_value(gripper_value, speed)

try:
    baud = 115200
    port = "/dev/ttyACM0"
    print(port, baud)

    mc = MyCobot(port, baud)
    running = True
    mc.set_fresh_mode(1)

    rospy.init_node('robot_sync_manager', anonymous=True)
    rospy.Rate(10)

    rospy.Subscriber("joint_states", JointState, callback, queue_size=1)
    print("Done subscribing")
    rospy.spin()

except rospy.ROSInterruptException:
    rospy.loginfo("Program interrupted before completion.")
