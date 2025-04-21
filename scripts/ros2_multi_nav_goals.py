import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import math
import time

## Instructions:
# export TURTLEBOT3_MODEL=waffle_pi
# ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=False map:=/HOME/WAREHOUSEBOTS_RS2/maps/Testmap2.yaml
# python3 ros2_multi_nav_goals.py

class MultiNavGoals(Node):
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__('multi_nav_goals')

        # Create a BasicNavigator object for high-level navigation tasks
        self.navigator = BasicNavigator()

        # Wait until the Nav2 stack is fully active
        self.navigator.waitUntilNav2Active()

        # List of navigation goals in the format (x, y, theta in degrees)
        self.goals = [
            (2.2, -0.43, 360),
            (2.3, 0.6, 270),
            (0.06, -0.34, 0),
            (2.2, -0.43, 360),
            (0.19, 0.15, 0)
        ]

        # Track total time taken for all goals
        self.total_time = 0.0

        # Send each goal in sequence
        self.send_goals()

    def send_goals(self):
        # Iterate over all the goals
        for idx, (x, y, theta_deg) in enumerate(self.goals):
            self.get_logger().info(f'Sending Goal {idx + 1}: x={x}, y={y}, θ={theta_deg}°')

            # Create a PoseStamped message from the x, y, theta
            goal_pose = self.create_pose(x, y, theta_deg)

            # Start the timer
            start_time = time.time()

            # Send the goal to the navigator and wait for it to complete
            self.navigator.goToPose(goal_pose)
            self.navigator.waitUntilNavComplete()

            # Stop the timer
            end_time = time.time()
            elapsed = end_time - start_time
            self.total_time += elapsed

            # Check the result status
            result = self.navigator.getResult()
            if result == self.navigator.Result.SUCCEEDED:
                self.get_logger().info(f"Goal {idx + 1} succeeded in {elapsed:.2f} seconds")
            else:
                self.get_logger().warn(f"Goal {idx + 1} failed or was canceled. Stopping further navigation.")
                break  # Stop if any goal fails

        # After all goals (or early stop), print total time and shutdown
        self.get_logger().info(f"All navigation goals processed. Total time: {self.total_time:.2f} seconds")
        rclpy.shutdown()

    def create_pose(self, x, y, theta_deg):
        # Create a PoseStamped message for the given coordinates and orientation
        pose = PoseStamped()
        pose.header.frame_id = 'map'  # Global map frame
        pose.header.stamp = self.get_clock().now().to_msg()

        # Set position
        pose.pose.position.x = x
        pose.pose.position.y = y

        # Convert Euler angle (theta) in degrees to a quaternion
        q = quaternion_from_euler(0, 0, math.radians(theta_deg))
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return pose


def main(args=None):
    # ROS 2 entry point
    rclpy.init(args=args)
    MultiNavGoals()
    rclpy.spin()


if __name__ == '__main__':
    main()
