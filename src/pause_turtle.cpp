#include <ros/ros.h>
#include <std_srvs/Empty.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pause_turtle_node");
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("pause");

	std_srvs::Empty srv;
	if (client.call(srv)) {
		ROS_INFO("> Pausing Turtle\n");
	} else {
		ROS_INFO("> Failed to call pause service\n");
	}
	return 0;
}