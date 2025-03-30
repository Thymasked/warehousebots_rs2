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
# rosrun WarehouseBots_RS2 nav_goals.py or just play the python script

import rospy
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal 
from tf.transformations import quaternion_from_euler
import numpy as np
from nav_msgs.msg import Odometry

class MultiNavGoals:
    def __init__(self): # Initialise everything
        rospy.init_node('multi_nav_goals', anonymous=False)

        # Create action client for move_base
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # Wait for the move_base action server to be available
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server()
        rospy.loginfo("Connected to move_base server")

        # Define list of goals (x, y, theta in degrees) --> Modify this to set goals
        self.goals = [
            (0.5, 2.0, 0), # Goal 2
            (-0.5, -0.3, 0), # Goal 3
            (2, -0.5, 360), # Goal 4
        ]

        # Initialise variables for total distance and time
        self.current_x = 0.0
        self.current_y = 0.0
        self.prev_x = 0.0
        self.prev_y = 0.0

        self.total_distance = 0.0
        self.total_time = 0.0

        # Subscribe to odometry topic
        rospy.Subscriber('/odom', Odometry, self.odomCallback)

        # Loop through the list of goals
        for goal in self.goals:
            if not self.pubGoals(goal[0], goal[1], goal[2]):
                rospy.logwarn("Goal failed. Stopping navigation.")
                break # Stop if a goal fails

        rospy.loginfo(f"Total Distance Traveled: {self.total_distance} meters")
        rospy.loginfo(f"Total Time Taken: {self.total_time} seconds")
        rospy.loginfo("All goals processed. Shutting Down.")

    # Odometry callback to update the robot's position
    def odomCallback(self, msg):
        # Get current position of from odometry
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Calculate the distance travelled from the previous position
        dx = self.current_x - self.prev_x
        dy = self.current_y - self.prev_y
        self.total_distance += np.sqrt(dx**2 + dy**2) # Euclidean distance

        # Update previous position
        self.prev_x = self.current_x
        self.prev_y = self.current_y

        # Log the distance traveled
        rospy.loginfo("Total distance traveled: %.2f meters", self.total_distance)

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
        # Record the time before sending goal
        start_time = rospy.get_time()
        self.move_base.send_goal(goal)

        # Wait for result
        self.move_base.wait_for_result()

        # Time to reach goal
        end_time = rospy.get_time()
        goal_time = end_time - start_time
        # Sum of time recorded for each goal
        self.total_time += goal_time

         # Check if goal succeeded
        state = self.move_base.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached!")
            rospy.loginfo(f"Time taken to reach goal: {goal_time} seconds")
            return True
        else:
            rospy.logwarn("Failed to reach goal!")
            return False

# Main Code to run class        
def main():
    try:
        MultiNavGoals() # Create instance of MultiNavGoals class
    except rospy.ROSInternalException:
        rospy.loginfo("Navigation interrupted.")  


if __name__ == '__main__':
    main()