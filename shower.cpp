#include "ros/ros.h"
#include "geometry_msgs/Vecotr3"


void trajectory_callback(const geometry_msgs::Vector3::ConstPtr *msg)
{
	ROS_INFO("X: [%f],  Y: [%f],  Z: [%f]", msg->x, msg->y, msg->z);
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "shower");
	ros::NodeHandle nh;

	ros::Subscriber shower_sub = nh.subscriber("aims/trajectory", 10, &trajectory_callback);
	ros::spin()
	return 0;
}

