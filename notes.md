**Session 21.01**
- kitchen repo contains several example nodes
- clone relevant nodes as subrepo with naming scheme group-name_node-name

***Usage***
- `roslaunch mycobot_280_gripper_moveit mycobot_moveit.launch` aswell as `roscore` need to run in background

# SSH Connection

- install ssh: `sudo apt update && sudo apt install openssh-server -y && sudo systemctl enable ssh && sudo systemctl start ssh`
- set state as up: `sudo ip link set ens33 up && sudo dhclient ens33`
- `ip a` shows ip address, should also show ens33 as state UP
- edit config: `sudo nano /etc/ssh/sshd_config` comment in the lines `Port 22 and ListenAddress 0.0.0.0`
- firewall: `sudo ufw allow 22/tcp && sudo ufw enable`
- start server: `sudo systemctl start ssh && sudo systemctl enable ssh`
- connect: `ssh -p 22 username@ip-address`

# Build

Long build time - possibly search better alternative, apriltags requires usage of `catkin_make_isolated`

1. `catkin_make_isolated`
2. `source devel/setup.bash`

# Camera Nodes

1. Launch nodes with `rosrun taskbot camera_node.py` and `rosrun taskbot apriltag_node.py`
2. Camera data is published to /camera/image_raw
3. Tags are published under tag_detections
4. pose_R = [
    [ 0.99183751  0.05636288 -0.11437471]  # First row: tag's x-axis
    [-0.03622316  0.9845915   0.17107737]  # Second row: tag's y-axis
    [ 0.12225478 -0.16553794  0.97859642]  # Third row: tag's z-axis
]
and
pose_T define the location in space
