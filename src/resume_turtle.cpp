#include <ros/ros.h>
#include <std_srvs/Empty.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "resume_turtle_node");
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("resume");

	std_srvs::Empty srv;
	if (client.call(srv)) {
		ROS_INFO("> Moving Turtle\n");
	} else {
		ROS_INFO("> Failed to call resume service\n");
	}
	return 0;
}