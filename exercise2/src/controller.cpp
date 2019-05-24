////////// Emanuele Aucone //////////

#include "ros/ros.h"
#include <iostream>
#include "geometry_msgs/Twist.h"
#include <exercise2/ControllerCommand.h>

ros::Publisher cmd_pub;
ros::Subscriber cmd_sub;

// Global variables
float long_velocity, yaw_rate;
geometry_msgs::Twist turtle_cmd;
// Command received flag
bool received = false;
// Saturation values
float min_long_vel, max_long_vel, min_yaw_rate, max_yaw_rate;

// Callback fro high level command
void commandCallback(const exercise2::ControllerCommand::ConstPtr& cmd_msg)
{
	long_velocity = cmd_msg->longitudinal_velocity;
	yaw_rate = cmd_msg->yaw_angular_velocity;
	received = true;
	std::cout << "Command received!" << std::endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle_controller");
	const double f = 10;    // Hz

	ros::NodeHandle nh("~");
	ros::Rate loop(f);

	// Set parameters included in the roslaunch command
	if(nh.getParam("SATmin_longVel", min_long_vel))
		ROS_INFO("Found parameter: min saturation value for longitudinal velocity set to %f m/s", min_long_vel);
	else
	{
		min_long_vel = -2.0;
		ROS_INFO("Parameter not found, min saturation value for longitudinal velocity set to -2 m/s");
	}	

	if(nh.getParam("SATmax_longVel", max_long_vel))
		ROS_INFO("Found parameter: max saturation value for longitudinal velocity set to %f m/s", max_long_vel);
	else
	{
		max_long_vel = 2.0;
		ROS_INFO("Parameter not found, max saturation value for longitudinal velocity set to 2 m/s");
	}	

	if(nh.getParam("SATmin_yawRate", min_yaw_rate))
		ROS_INFO("Found parameter: min saturation value for yaw rate set to %f rad/s", min_yaw_rate);
	else
	{
		min_yaw_rate = -1.0;
		ROS_INFO("Parameter not found, min saturation value for yaw rate set to - 1 rad/s");
	}	

	if(nh.getParam("SATmax_yawRate", max_yaw_rate))
		ROS_INFO("Found parameter: max saturation value for yaw rate set to %f rad/s", max_yaw_rate);
	else
	{
		max_yaw_rate = 1.0;
		ROS_INFO("Parameter not found, max saturation value for yaw rate set to 1 rad/s");
	}	

	// Publisher and subscriber
	cmd_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	cmd_sub = nh.subscribe("/controller/high_level_command", 10, &commandCallback);

	sleep(1);
	std::cout << "\r\n\n\n\033[32m\033[1mSend high level command! \033[0m" << std::endl;

	while(ros::ok())         
	{
		if(received)
		{
			// Apply saturation on longitudinal velocity
			if(long_velocity <= min_long_vel)
				turtle_cmd.linear.x = min_long_vel;
			else if(long_velocity >= max_long_vel)
				turtle_cmd.linear.x = max_long_vel;
			else
				turtle_cmd.linear.x = long_velocity;

			// Apply saturation on yaw rate
			if(yaw_rate <= min_yaw_rate)
				turtle_cmd.angular.z = min_yaw_rate;
			else if(yaw_rate >= max_yaw_rate)
				turtle_cmd.angular.z = max_yaw_rate;
			else
				turtle_cmd.angular.z = yaw_rate;

			//Publish on turtlesim node
			cmd_pub.publish(turtle_cmd);
			received = false;
			std::cout << "\r\n\n\n\033[32m\033[1mSend another high level command! \033[0m" << std::endl;
		}

		ros::spinOnce();
		loop.sleep();          
	}
	return 0;
}
