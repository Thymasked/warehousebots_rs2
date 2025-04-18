#include <ros/ros.h> 
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h> 
#include <opencv2/opencv.hpp> 
#include <cv_bridge/cv_bridge.h> 

//RS2 Turtlebot Warehouse Setup
// By :
// This code is about....

class taskManagerNode 

{
public:
    taskManagerNode() 
    {
        // Initialize subscribers and publishers

	//subImage = nh.subscribe("/usb_cam/image_raw",10, &taskManagerNode::processImage, this); // Read image data from Logitech C930e (USE FOR REAL TURTLEBOT)
	
	// LiDAR subscribe
        
        //subImage = nh.subscribe("/camera/rgb/image_raw",10, &taskManagerNode::processImage, this); // Read image data from the TurtleBot Camera feed (Simulation Testing)
        
        pubVelocity = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10); // Send velocity commands to control TurtleBot's motors
        
        ROS_INFO("taskManager Node initialized."); // Indicate if the code is running
    }

private:
    ros::NodeHandle nh; 
    ros::Subscriber subImage; 
    ros::Publisher pubVelocity; 
    

    // Function to publish the velocity command to the robot
    void publishVelocity(double linear, double angular) {
        geometry_msgs::Twist msg;
        msg.linear.x = linear; 
        msg.angular.z = angular; 
        pubVelocity.publish(msg); // Publish message to "/cmd_vel" topic to control robot movement
    }
};


int main(int argc, char** argv) { 
    ros::init(argc, argv, "wbots_main"); 
    taskManagerNode node; 
    ros::spin(); // Enter a loop to keep ROS node running and processing incoming messages
    return 0; 
}
