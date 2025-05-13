# Install all dependencies
# Install turtlebot4 packages

# ROS2 add discovery server to remote pc bash file:
# $ wget -qO - https://raw.githubusercontent.com/turtlebot/turtlebot4_setup/humble/turtlebot4_discovery/configure_discovery.sh | bash <(cat) </dev/tty
# ROS_DOMAIN_ID [0]: "enter"
# Discovery Server ID [0]: "enter"
# Discovery Server IP: 192.168.0.70
# Discovery Server Port [11811]: 11811
# Press done: d

# Bring up the turtlebot4 through ssh ubuntu@192.168.0.70:
# ros2 launch turtlebot4_bringup robot.launch.py

# Run teleop on remote pc (OBVIOUS CHOICE):
# ros2 run teleop_twist_keyboard teleop_twist_keyboard 

# Connect bluetooth joy stick controller (TOO DIFFICULT RIGHT NOW):
# https://turtlebot.github.io/turtlebot4-user-manual/setup/basic.html#turtlebot-4-controller-manual-setup
# connect A0:5A:5C:E5:67:5C
# ros2 launch turtlebot4_bringup joy_teleop.launch.py namespace:=/

#source ~/turtlebot4_ws/install/setup.bash
#source /etc/turtlebot4_discovery/setup.bash