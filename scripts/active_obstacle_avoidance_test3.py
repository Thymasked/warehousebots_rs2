#!/usr/bin/env python3

# Time Synchronisation: ssh ubuntu@192.168.0.210 "sudo date --set='$(date +"%Y-%m-%d %H:%M:%S")'"

# Coded by Daniel Nguyen - WarehouseBots - Robotic Studio 2
# Test 3: Active Obstacle Avoidance
# Structure incorporated from turtlebot3/turtlebot3_example/nodes/turtlebot3_point_key --> But use move_base topic instead of cmd_vel (including navigation stack to handle obstacle avoidance)

# For better obstacle avoidance and smooth faster path replanning, edit:
# Global Planner, DWA Local Planner, Costmap Settings and Recovery Behaviour
# Increase planner_frequency, increase controller_frequency, decrease sim_time, adjust (lower to be more responsice to obstacles) 
# path_distance_bias and goal_distance_bias, increase update_frequency and publish_frequency (10.0 and 5.0)
# adjust recovery_attempts and clearing_radius (3 attempts and 1.0 radius)

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

class ActiveObsAvoidance:
    def __init__(self): # Initialise everything
        rospy.init_node('active_obs_avoidance', anonymous=False)

        # Create action client for move_base
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # Wait for the move_base action server to be available
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server()
        rospy.loginfo("Connected to move_base server")

        # Define list of goals (x, y, theta in degrees) --> Modify this to set goals (Ensure it is within map range)
        # Check the start pose of the turtlebot3 using 'rostopic echo /amcl_pose'

        self.goals = [
            (0.10, -1.34, 0)] # Goal Position
        
        self.goalReturn = [
            (0.98, -2.67, 360) # Start Position
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

        # Start the time
        self.start_time = rospy.get_time()

        # Loop through the list of goals
        for goal in self.goals:
            if not self.pubGoals(goal[0], goal[1], goal[2]):
                rospy.logwarn("Goal failed. Stopping navigation.")
                continue # Skip to the next goal if the current one fails

        # Wait for 10 seconds for other robot to finish before returning
        rospy.loginfo("Waiting for 5 seconds before returning to start position...")
        rospy.sleep(10)

        # Return to the original position
        if not self.pubGoals(self.goalReturn[0][0], self.goalReturn[0][1], self.goalReturn[0][2]):
            rospy.logwarn("Returning to start position failed.")

        self.end_time = rospy.get_time()
        self.total_time = self.end_time - self.start_time

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
        #start_time = rospy.get_time()
        self.move_base.send_goal(goal)

        # Wait for result
        success = self.move_base.wait_for_result(rospy.Duration(60.0)) # Time out in seconds

        if not success:
            rospy.logwarn(f"Goal failed due to timeout or other issues.")
            return False

        # Time to reach goal
        #end_time = rospy.get_time()
        #goal_time = end_time - start_time

        # Sum of time recorded for each goal
        #self.total_time += goal_time

        # Retry if goal failed
        retries = 2  # number of retries for a failed goal

        # Check if goal succeeded
        state = self.move_base.get_state()
        while state != actionlib.GoalStatus.SUCCEEDED and retries > 0:
            rospy.logwarn(f"Goal failed. Retrying... {retries} attempts left.")
            self.move_base.send_goal(goal)
            success = self.move_base.wait_for_result(rospy.Duration(60.0)) 
            state = self.move_base.get_state()
            retries -= 1 
            
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached!")
            rospy.loginfo(f"Time taken to reach goal: {goal_time} seconds")
            return True
        else:
            rospy.logwarn("Failed to reach goal after multiple attempts.")
            return False

# Main Code to run class        
def main():
    try:
        ActiveObsAvoidance() # Create instance of MultiNavGoals class
    except rospy.ROSInternalException:
        rospy.loginfo("Navigation interrupted.")  


if __name__ == '__main__':
    main()

