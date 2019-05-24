////////// Emanuele Aucone //////////

#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <turtlesim/Pose.h>
#include <stdlib.h>

// Global variables
geometry_msgs::Twist turtle_cmd;
float back_vel = -1.0;
int seed = 2019;
bool avoidance = false;

// Callback for turtle pose and object detection
void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
	if(/*msg->x <= 1.0 || msg->x >= 10.0 ||*/ msg->y <= 5.5 /*|| msg->y >= 10.0*/)
	{
		avoidance = true;
		// Going backward
		ROS_INFO("Obstacle detected");
		if(msg->linear_velocity > 0)
			turtle_cmd.linear.x = back_vel;
		else
			turtle_cmd.linear.x = -back_vel;
	}
	else 
		avoidance = false;	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "random_controller");
	const double f = 1;    // Hz

	ros::NodeHandle nh;
	ros::Rate loop(f);

	ros::Publisher random_cmd_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	ros::Subscriber pose_sub = nh.subscribe("/turtle1/pose", 10, &poseCallback);

	while(ros::ok())         
	{
		// Check if current position is inside C_free
		ros::spinOnce();

		if(!avoidance)
		{
			// Set the seed for random generation of cmd_vel
			srand(seed);
			seed = rand() % 100000000;
			// Move the turtle with random command (bound: [-3/+3] m/s and -[3,+3] rad/s)
			turtle_cmd.linear.x = (rand()%60 - 30)/10;
			turtle_cmd.angular.z = (rand()%30 - 30)/10;
			random_cmd_pub.publish(turtle_cmd);
		}
		else
		{
			// Send the command corrected in the callback
			random_cmd_pub.publish(turtle_cmd);
			sleep(1);
		}
		loop.sleep();
	}
	return 0;
}
