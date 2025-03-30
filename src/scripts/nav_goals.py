#!/usr/bin/env python

# Coded by Daniel Nguyen - WarehouseBots - Robotic Studio 2
# TEST 1: Consistent Goals
# Structure incorporated from turtlebot3/turtlebot3_example/nodes/turtlebot3_point_key --> But use move_base topic instead of cmd_vel (including navigation stack to handle obstacle avoidance)

#----------  Instructions to run code in simulation --------#
# export TURTLEBOT3_MODEL=waffle_pi
# roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch 
# export TURTLEBOT3_MODEL=waffle_pi
# roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
# make the script executable: chmod +x nav_goals.py
# rosrun WarehouseBots_RS2 nav_goals.py

#----------- Real Robot Instructions --------#
# roscore
# export TURTLEBOT3_MODEL=waffle_pi (ssh)
# roslaunch turtlebot3_bringup turtlebot3_robot.launch 
# sort out map and slam
# export TURTLEBOT3_MODEL=waffle_pi
# roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
# localise the robot
# rosrun WarehouseBots_RS2 nav_goals.py


import rospy
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal 
from tf.transformations import quaternion_from_euler
import numpy as np

class MultiNavGoals:
    def _init_(self): # Initialise everything
        rospy.init_node('multi_nav_goals', anonymous=False)

        # Create action client for move_base
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # Wait for the move_base action server to be available
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server()
        rospy.loginfo("Connected to move_base server")

        # Define list of goals (x, y, theta in degrees) --> Modify this to set goals
        self.goals = [
            (0.5, 1.0, 0), # Goal 1
            (1.0, 2.0, 0), # Goal 2
            (1.5, 0.5, 0), # Goal 3
            (2.5, 1.5, 0), # Goal 4
        ]

        # Loop through the list of goals
        for goal in self.goals:
            if not self.goals(goal[0], goal[1], goal[2]):
                rospy.logwarn("Goal failed. Stopping navigation.")
                break # Stop if a goal fails

        rospy.loginfo("All goals processed. Shutting Down.")

    def pubGoals(self, x, y, theta):
        # Sends a goal to move_base and wait for result
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"  # Use 'map' frame for global navigation
        goal.target_pose.header.stamp = rospy.Time.now()

        # Set position
        goal.target_pose.pose.position = Point(x, y, 0.0)

        # Convert θ (degrees) to quaternion
        q = quaternion_from_euler(0, 0, np.deg2rad(theta))
        goal.target_pose.pose.orientation = Quaternion(*q)

        # Send goal to move_base
        rospy.loginfo(f"Sending goal: x={x}, y={y}, θ={theta}°")
        self.move_base.send_goal(goal)

         # Check if goal succeeded
        state = self.move_base.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached!")
            return True
        else:
            rospy.logwarn("Failed to reach goal!")
            return False

if __name__ == '__main__':
    try:
        MultiNavGoals()
    except rospy.ROSInternalException:
        rospy.logininfo("Navigation interrupted.")