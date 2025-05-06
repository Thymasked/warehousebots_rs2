#!/usr/bin/env python3

# WARNING: WORK ONLY ON ROS1

# Time Synchronisation: ssh ubuntu@192.168.0.210 "sudo date --set='$(date +"%Y-%m-%d %H:%M:%S")'"
# Manual: sudo date -s "2025-05-06 15:08:50"

# Coded by Daniel Nguyen - WarehouseBots - Robotic Studio 2
# TEST 2: A* vs Dijkstra Path Planning
# Structure incorporated from turtlebot3/turtlebot3_example/nodes/turtlebot3_point_key --> But use move_base topic instead of cmd_vel (including navigation stack to handle obstacle avoidance)

# Install plugins:
# sudo apt-get install ros-noetic-global-planner
# Make sure to edit the move_base_params.yaml file for planner type before running navigation


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
import csv
import os
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
import numpy as np
from nav_msgs.msg import Odometry
import time

class PathOptimizationTest:
    def __init__(self, trial_num):
        rospy.init_node(f'path_optimization_test_{trial_num}', anonymous=False)

        # Create action client for move_base
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # Wait for the move_base action server to be available
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server()
        rospy.loginfo("Connected to move_base server")

        # Define list of goals (x, y, theta in degrees) --> Modify this to set goals (Ensure it is within map range)
        self.goals = [
            (-0.523, 0.578, 90), # Goal 1
            (-1.28, 1.86, 360), # Goal 2
            (-0.271, 2.05, 360), # Goal 3
        ]

        # Initialise variables for total distance and time
        self.current_x = 0.0
        self.current_y = 0.0
        self.prev_x = 0.0
        self.prev_y = 0.0

        self.total_distance = 0.0
        self.total_time = 0.0
        self.deviation = 0.0

        # Subscribe to odometry topic
        rospy.Subscriber('/odom', Odometry, self.odomCallback)

        # Loop through the list of goals
        for goal in self.goals:
            if not self.pubGoals(goal[0], goal[1], goal[2]):
                rospy.logwarn("Goal failed. Moving to next goal.")
                continue # Skip to the next goal if the current one fails

        rospy.loginfo(f"Total Distance Traveled: {self.total_distance} meters")
        rospy.loginfo(f"Total Time Taken: {self.total_time} seconds")

        rospy.loginfo("All goals processed. Shutting Down.")

        # Call deviation calculator
        self.deviation = self.calculate_deviation()

        # Return the robot back to starting pose
        rospy.loginfo("Returning back to starting position...")
        home_success = self.pubGoals(0.078, -0.638, 0.0) # Edit this as starting pose
        
        if home_success:
            rospy.loginfo("Returned to origin successfully.")
        else:
            rospy.loginfo("Failed to return to origin")


    # Odometry callback to update the robot's position
    def odomCallback(self, msg):
        # Get current position of from odometry
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Calculate the distance travelled from the previous position
        dx = self.current_x - self.prev_x
        dy = self.current_y - self.prev_y
        self.total_distance += np.sqrt(dx**2 + dy**2)  # Euclidean distance

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
        rospy.loginfo(f"[{rospy.get_time():.0f}s] Sending goal: x={x}, y={y}, θ={theta}°")

        # Retry if goal failed
        retries = 2 # Number of retries

        # Loop to check if goal is unsuccessful and retries is more than 0
        while retries >= 0:
            # Record the time before sending goal
            start_time = rospy.get_time()
            self.move_base.send_goal(goal)
            self.move_base.wait_for_result(rospy.Duration(60.0)) 
            end_time = rospy.get_time()

            state = self.move_base.get_state()      
            if state == actionlib.GoalStatus.SUCCEEDED:
                # Time to reach goal
                goal_time = end_time - start_time
                # Sum of time recorded for each goal
                self.total_time += goal_time

                rospy.loginfo("Goal reached!")
                rospy.loginfo(f"Time taken to reach goal: {goal_time:.2f} seconds")
                return True
            else:
                rospy.logwarn(f"Failed to reach goal. Retrying... {retries} attempts left.")
                retries -= 1
        rospy.logwarn("Failed to reach goal after multiple attempts.")
        return False
                
        
    # Calculate distance deviation
    def calculate_deviation(self):
        #human_path_length = 0.64 # Measure in meters, update if needed
        optimal_path_dist = self.optimalPathLength()
        self.deviation = ((self.total_distance - optimal_path_dist) / optimal_path_dist) * 100
        rospy.loginfo(f"Path deviation: {self.deviation:.2f}%")
        return self.deviation
    
    # May need to add a function for calculating optimal path length for all goals
    def optimalPathLength(self):
        total_optimal_distance = 0.0
        for i in range(len(self.goals)-1):
            x1, y1, _ = self.goals[i]
            x2, y2, _ = self.goals[i + 1]
            dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            total_optimal_distance += dist
        rospy.loginfo(f"Optimal (shortest) path length: {total_optimal_distance:.2f} meters")
        return total_optimal_distance

    # Refining the PARAMETERS ensures smaller deviations
    # goal_distance_bias (increase value for smoother trajectories and efficient movement in open spaces)
    # occdist_scale (increase value to make robot more cautious and less likely to take risky paths and better obstacle avoidance)
    # plan_cost_scale (Tuning minimises the total cost of path, including distance and obstacle avoidance, balance robot speed and efficiency with its ability to 
    # navigate complex environments) 

# Main Code to run class        
def main():
    # Initialise results file
    results_file = "path_planning_results.csv"

    # Create the CSV file and write header only if it doesn't exist
    if not os.path.exists(results_file):
        with open(results_file, mode='w') as f:
            writer = csv.writer(f)
            writer.writerow(['Trial', 'Distance(m)', 'Time(s)', 'Path Deviation (%)'])

    try:
        # Get planner type from the parameter server (defaults to 'A*' if not set)
        planner_type = rospy.get_param('/move_base/GlobalPlanner/plan_type', 'A*')

        # Log the chosen planner type
        rospy.loginfo(f"Using {planner_type} planner for path planning tests.")

        # Run one trial for the selected planner (A* or Dijkstra)
        rospy.loginfo(f"\n=== Running Test for {planner_type} Planner ===")
        path_test = PathOptimizationTest(1)  # Use a single trial with dynamic planner type
        rospy.sleep(2)  # Wait briefly before next run

        # Save results to CSV
        with open(results_file, mode='a') as f:
            writer = csv.writer(f)
            writer.writerow([1, f"{path_test.total_distance:.2f}", f"{path_test.total_time:.2f}", f"{path_test.deviation:.2f}"])

    except rospy.ROSInternalException:
        rospy.loginfo("Navigation interrupted.")  


if __name__ == '__main__':
    main()