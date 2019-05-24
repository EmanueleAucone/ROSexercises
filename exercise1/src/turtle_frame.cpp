////////// Emanuele Aucone //////////

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

// Callback to acquire turtle pose and generate fixed and local frames
void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
	// Create broadcaster trasform
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	// Set the starting point of the frame
	transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, msg->theta);
	// Compute the frame trasformation
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "turtle"));
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "turtle_frame");
	
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/turtle1/pose", 10, &poseCallback);

	ros::spin();
	return 0;
}
