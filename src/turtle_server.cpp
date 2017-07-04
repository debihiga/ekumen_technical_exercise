
#include "ekumen_technical_exercise/turtle_server.hpp"

void TurtleServer::executeCb(const ekumen_technical_exercise::TurtleGoalConstPtr &goal) {
	// TODO
	ROS_INFO("\n> executeCb()\n");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtle_server_node");

  TurtleServer server(ros::this_node::getName());

  ros::spin();
  
  return 0;
}
