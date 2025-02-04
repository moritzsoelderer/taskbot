**Session 21.01**
- kitchen repo contains several example nodes
- clone relevant nodes as subrepo with naming scheme group-name_node-name

***Usage***
- rosrun *.py instead of python *.py
- `roslaunch mycobot_280_gripper_moveit mycobot_moveit.launch` aswell as `roscore` need to run in background 

# SSH Connection

- install ssh: `sudo apt update && sudo apt install openssh-server -y && sudo systemctl enable ssh && sudo systemctl start ssh`
- set state as up: `sudo ip link set ens33 up && sudo dhclient ens33`
- `ip a` shows ip address, should also show ens33 as state UP
- edit config: `sudo nano /etc/ssh/sshd_config` comment in the lines `Port 22 and ListenAddress 0.0.0.0`
- firewall: `sudo ufw allow 22/tcp && sudo ufw enable`
- start server: `sudo systemctl start ssh && sudo systemctl enable ssh`