#!/bin/bash

# Make Python scripts executable
sudo find ~/catkin_ws/src/taskbot/scripts/ -name "*.py" -exec chmod +x {} \;

# Change permissions on tty devices only if the previous one failed
sudo find /dev/ -name "ttyACM0" -exec chmod 777 {} \;
sudo find /dev/ -name "ttyACM1" -exec chmod 777 {} \;
sudo find /dev/ -name "ttyUSB1" -exec chmod 777 {} \;

# Launch ROS launch file
(roslaunch taskbot taskbot_moveit.launch)
