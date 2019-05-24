////////// Emanuele Aucone //////////

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <exercise4/TurtleStatus.h>
#include <string>
#include <stdlib.h>
#include <iostream>
#include <math.h>

// Publishers and subscribers
ros::Subscriber turtle_status_sub;
ros::Subscriber turtle_pose_sub;
ros::Publisher turtle_status_pub;
ros::Publisher vel_pub;

geometry_msgs::Twist turtle_cmd;

// Global variables
struct State {
	std::string ID;
	turtlesim::Pose pose;
	// It's necessary to define "operator <" if I want to use the struct in the map
	bool operator<(const State& s) const
	{ 
		return (this->ID < s.ID); 
	} 
} ;
State s;
std::map<State,bool> turtles_state;

// Node variable
int turtle_ID;
std::string turtle_name;
const int turtle_number = 4;
int first_read = 1;

// Motion controller parameters
const float tolerance = 0.15;
const float K_v = 0.2;
const float K_w = 0.5;

// Flags
bool position_read = false;
bool position_arrived = false;
bool motion = false;
bool start_move = false;

// Variables for the turtles motion
float curr_x, curr_y, curr_theta;	// current position
float des_x, des_y;			// desired position -> baricentre
float steering_angle;			// steering angle used to yaw rate control

// Read its own position 
void turtlePoseCallback(const turtlesim::Pose::ConstPtr& pose_msg)
{
	curr_x = pose_msg->x;
	curr_y = pose_msg->y;
	curr_theta = pose_msg->theta;
	if(first_read == 1)
	{
		position_read = true;
		first_read++;
	}
}

// Team Callback
void turtleStatusCallback(const exercise4::TurtleStatus::ConstPtr& status_msg)
{
	if(position_read)
	{
		// Check if positions are already sent to the team
		if(position_arrived) return;
	
		s = {status_msg->turtle_id, status_msg->pose};
	  	turtles_state[s] = status_msg->position_sent;

		int counter = 0;
		for(auto turtle : turtles_state)
		{
			if(turtle.second) counter++;
		}

		if(counter == turtle_number){
			ROS_INFO_STREAM("Turtles team is ready. Positions received by " << turtle_name);
			position_arrived = true;
		}
	}
	if(motion)
	{
		s = {status_msg->turtle_id, status_msg->pose};
	  	
		float prev_id = s.ID.back() - '0';
		if((turtle_ID == 1) || ((prev_id+1) == turtle_ID && status_msg->arrived))
			start_move = true;
	}
}

// Publish on team topic its own position
void publishOwnPosition(std::string turtle_name_)
{
	if(position_read)
	{	
		exercise4::TurtleStatus status_msg;

		// Send info to the turtles team
		status_msg.header.stamp = ros::Time::now();
		status_msg.turtle_id = turtle_name_;
		status_msg.pose.x = curr_x;
		status_msg.pose.y = curr_y;
		status_msg.pose.theta = curr_theta;
		status_msg.position_sent = true;
		status_msg.arrived = false;

		// Wait for the publisher to connect to subscribers
		sleep(2.0);
		turtle_status_pub.publish(status_msg);

		std::cout << "Position published by " << turtle_name_ 
			  << ": x = "		      << curr_x 
			  << ", y = "  		      << curr_y << std::endl;
	}
}

// Publish on team topic to inform if the goal is achieved
void publishArrive(std::string turtle_name_)
{
	exercise4::TurtleStatus status_msg;
	status_msg.header.stamp = ros::Time::now();
	status_msg.turtle_id = turtle_name_;
	status_msg.pose.x = curr_x;
	status_msg.pose.y = curr_y;
	status_msg.pose.theta = curr_theta;
	status_msg.position_sent = true;
	//status_msg.arrived = false;

	// Wait for the publisher to connect to subscribers
	sleep(1.0);
	turtle_status_pub.publish(status_msg);
}

// Calculate the position of the baricentre using the position contained in the map
void calculate_baricentre()
{
	float sum_x = 0;
	float sum_y = 0;
	for(auto turtle : turtles_state)
	{
		sum_x = sum_x + turtle.first.pose.x;
		sum_y = sum_y + turtle.first.pose.y;
	}
	des_x = sum_x/turtle_number;
	des_y = sum_y/turtle_number;
}

// Wait until all turtles have sent their own position
void waitForTeamPose()
{
	ros::Rate loop(1);

	while (!position_arrived)
	{
		ROS_INFO_STREAM("Turtle waiting for team: " << turtle_name);
		publishOwnPosition(turtle_name);
		// Call the turtleStatusCallback
		ros::spinOnce();	
		loop.sleep();
	}
	position_read = false;
}

// Calculate the euclidean distance
float distance(float des_X, float des_Y, float curr_X, float curr_Y)
{
	return sqrt(pow(des_X-curr_X, 2) + pow(des_Y-curr_Y, 2));
}

// Proportional controller implemented to let the turtle reaches the desired position
void move_turtles()
{
	motion = false;
	while(distance(des_x, des_y, curr_x, curr_y) >= tolerance)
	{
		steering_angle = atan2(des_y - curr_y, des_x - curr_x);
		if(steering_angle > 2*M_PI) steering_angle -= 2*M_PI;
		else if(steering_angle < -2*M_PI) steering_angle += 2*M_PI;

		turtle_cmd.linear.x = K_v * distance(des_x, des_y, curr_x, curr_y);
		turtle_cmd.angular.z = K_w * (steering_angle - curr_theta);

		vel_pub.publish(turtle_cmd);
		ros::spinOnce();
		sleep(1.0);
	}
	// Stop the turtle
	std::cout << turtle_name << " is arrived!" << std::endl;
	turtle_cmd.linear.x = 0;
	turtle_cmd.angular.z = 0;
	vel_pub.publish(turtle_cmd);

	// Let the team know theturtle is arrived to the desired position
	exercise4::TurtleStatus status_msg;
	status_msg.header.stamp = ros::Time::now();
	status_msg.turtle_id = turtle_name;
	status_msg.pose.x = curr_x;
	status_msg.pose.y = curr_y;
	status_msg.pose.theta = curr_theta;
	status_msg.position_sent = true;
	status_msg.arrived = true;
	turtle_status_pub.publish(status_msg);
	start_move = false;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle_team");
	ros::NodeHandle nh("~");
	ros::Rate loop(10);

	// Turtle name
	nh.getParam("turtle_name", turtle_name);
	turtle_ID = turtle_name.back() - '0';

	// Publish and subscribe to turtles team status messages
	turtle_status_pub = nh.advertise<exercise4::TurtleStatus>("/turtle_status", 10);
	turtle_status_sub = nh.subscribe("/turtle_status", 20, &turtleStatusCallback);

	turtle_pose_sub = nh.subscribe("/" + turtle_name + "/pose", 20, &turtlePoseCallback);
	vel_pub = nh.advertise<geometry_msgs::Twist>("/" + turtle_name + "/cmd_vel", 10);

	// Call the turtlePoseCallback to get current position
	ros::spinOnce();	
	sleep(1.0);

	// Each turtle publishes its own position
	publishOwnPosition(turtle_name);

	// Wait for turtles team
	waitForTeamPose();

	// Compute the baricentre
	calculate_baricentre();
	std::cout << "\r\n\033[32m\033[1mBaricentre coordinates  (desired goal position):\033[0m"
		  << " x = "  << des_x 
		  << ", y = " << des_y << std::endl;

	position_read = false;
	motion = true;

	std::cout << "\r\n\033[32m\033[1mLet's start moving! \033[0m" << std::endl;

	// Move the turtle one at a time
	while(ros::ok())         
	{
		publishArrive(turtle_name);
		// Check if the previous turtle is arrived through teamStatusCallback
		ros::spinOnce();
		
		if(start_move)
			move_turtles();
		
		loop.sleep();          
	}

	return 0;
}
