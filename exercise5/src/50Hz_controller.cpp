////////// Emanuele Aucone //////////

#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <turtlesim/Pose.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>

// Publishers and subscribers
ros::Publisher cmd_pub;
ros::Subscriber pose_sub;
ros::Subscriber pose_sub_from_turtle2;
ros::Subscriber pose_sub_from_turtle3;

geometry_msgs::Twist turtle_cmd;

// Turtles variables
float curr_x1, curr_y1, curr_theta1;
float x2, y2, theta2;
float x3, y3, theta3;
float des_x, des_y;

// Controller variables
float tolerance = 0.15;
float tolerance_obstacles = 1.0;
float K_v = 0.1;
float K_w = 0.3;
float steering_angle;

// Obstacle avoidance variables
float back_vel = -0.3;
int t = 1;
float linear_vel;
float angular_vel;

// Flags
int count[3] = {0, 0, 0};
bool first1 = true;
bool first2 = true;
bool first3 = true;
bool done = false;
bool obstacle = false;

// Callback to get turtle1 pose 
void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
	curr_x1 = msg->x;
	curr_y1 = msg->y;
	curr_theta1 = msg->theta;
	linear_vel = msg->linear_velocity;
	angular_vel = msg->angular_velocity;
	if(first1)
	{
		std::cout << "Turtle1 initial pose: x = " << curr_x1 << ", y = " << curr_y1 << std::endl;
		count[0] = 1;
		first1 = false;
	}
}

// Callbacks to get other turtles position
void pose2Callback(const turtlesim::Pose::ConstPtr& msg)
{
	if(first2)
	{
		x2 = msg->x;
		y2 = msg->y;
		theta2 = msg->theta;
		std::cout << "Turtle2 pose: x = " << x2 << ", y = " << y2 << std::endl;
		count[1] = 1;
		first2 = false;
	}
}

void pose3Callback(const turtlesim::Pose::ConstPtr& msg)
{
	if(first3)
	{
		x3 = msg->x;
		y3 = msg->y;
		theta3 = msg->theta;
		std::cout << "Turtle3 pose: x = " << x3 << ", y = " << y3 << std::endl;
		count[2] = 1;
		first3 = false;
	}
}

// Wait until all positions are received
void waitForTeamPose()
{
	while(count[0] == 0 || count[1] == 0 || count[2] == 0)
	{
		ros::spinOnce();	
		sleep(1.0);
	}
}

// Calculate the euclidean distance
float distance(float p2_x, float p2_y, float p1_x, float p1_y)
{
	return sqrt(pow(p2_x - p1_x, 2) + pow(p2_y - p1_y, 2));
}

// Avoid other turtles considering them as obstacles
void obstacleAvoidance()
{
	if((distance(x2, y2, curr_x1, curr_y1) <= tolerance_obstacles) || (distance(x3, y3, curr_x1, curr_y1) <= tolerance_obstacles))
	{
		if(distance(x2, y2, curr_x1, curr_y1) <= tolerance_obstacles)
			t = 2;
		else if(distance(x3, y3, curr_x1, curr_y1) <= tolerance_obstacles)
			t = 3;
		// Correct the direction of motion
		std::cout << "Obstacle detected: turtle" << t << std::endl;
		obstacle = true;
		ros::spinOnce();
		turtle_cmd.linear.x = -back_vel;
		//sleep(1.0);
		if(t == 2)
		{
			if((curr_x1 < x2 && curr_y1 > y2) || (curr_x1 > x2 && curr_y1 < y2))
				turtle_cmd.angular.z = 1.2*sqrt(pow(curr_theta1,2));
			else if((curr_x1 < x2 && curr_y1 < y2) || (curr_x1 > x2 && curr_y1 > y2))
				turtle_cmd.angular.z = -1.2*sqrt(pow(curr_theta1,2));	
		}
		else if(t == 3)
		{
			if((curr_x1 < x3 && curr_y1 > y3) || (curr_x1 > x3 && curr_y1 < y3))
				turtle_cmd.angular.z = 1.2*sqrt(pow(curr_theta1,2));
			else if((curr_x1 < x2 && curr_y1 < y3) || (curr_x1 > x2 && curr_y1 > y3))
				turtle_cmd.angular.z = -1.2*sqrt(pow(curr_theta1,2));	
		}
		cmd_pub.publish(turtle_cmd);			
		sleep(1.0);
	}
	else
		obstacle = false;
}

// Proportional controller implemented to let the turtle reaches the desired position
void move_turtle()
{
	steering_angle = atan2(des_y - curr_y1, des_x - curr_x1);
	
	if(steering_angle > 2*M_PI) steering_angle -= 2*M_PI;
	else if(steering_angle < -2*M_PI) steering_angle += 2*M_PI;
		
	if(!obstacle){
		turtle_cmd.linear.x = K_v * distance(des_x, des_y, curr_x1, curr_y1);
		turtle_cmd.angular.z = K_w * (steering_angle - curr_theta1);

		cmd_pub.publish(turtle_cmd);
	}

	sleep(1.0);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "controller");
	const double f = 50;    // Hz

	ros::NodeHandle nh("~");
	ros::Rate loop(f);

	// Set finishing point to be reached
	if(nh.getParam("desired_x", des_x))
		ROS_INFO("Found parameter: %f", des_x);
	else
	{
		des_x = 9;
		ROS_INFO("Parameter not found");
	}	

	if(nh.getParam("desired_y", des_y))
		ROS_INFO("Found parameter: %f", des_y);
	else
	{
		des_y = 2.0;
		ROS_INFO("Parameter not found");
	}	

	cmd_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	
	pose_sub_from_turtle2 = nh.subscribe("/turtle2/pose", 1, &pose2Callback);
	pose_sub_from_turtle3 = nh.subscribe("/turtle3/pose", 1, &pose3Callback);
	pose_sub = nh.subscribe("/turtle1/pose", 10, &poseCallback);

	// Acquire turtles positions
	waitForTeamPose();

	while(ros::ok())         
	{
		while(distance(des_x, des_y, curr_x1, curr_y1) >= tolerance)
		{
			// Check if current position is inside C_free and correct if necessary
			ros::spinOnce();
			obstacleAvoidance();

			if(distance(des_x, des_y, curr_x1, curr_y1) <= 1.0)
				K_v = 0.8;	// Speed up the controller gain
			move_turtle();
			
			loop.sleep();
		}
		if(!done)
		{
			// Stop the turtle
			std::cout << "turtle1 is arrived!" << std::endl;
			turtle_cmd.linear.x = 0;
			turtle_cmd.angular.z = 0;
			cmd_pub.publish(turtle_cmd);
			done = true;
		}
	}
	return 0;
}
