#!/bin/bash

echo "Starting TurtleBot3 SLAM workflow using Terminator..."

# Start Terminator and split into 4 terminals
terminator --new-tab -e "bash -c 'roscore'" &
sleep 10

echo "Connecting to TurtleBot via SSH..."
sshpass -p "turtlebot" ssh -o StrictHostKeyChecking=no ubuntu@192.168.0.210 << 'EOF'
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_bringup turtlebot3_robot.launch
EOF
sleep 10

terminator --new-tab -e "bash -c 'export TURTLEBOT3_MODEL=waffle_pi; roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=hector; exec bash'" &
sleep 10

terminator --new-tab -e "bash -c 'export TURTLEBOT3_MODEL=waffle_pi; roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch; exec bash'" &

echo "TurtleBot3 SLAM workflow started successfully!"

