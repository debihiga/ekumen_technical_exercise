#include <ros/ros.h>
//#include "ekumen_technical_exercise/ekumen_technical_exercise.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ekumen_technical_exercise");
  ros::NodeHandle nodeHandle("~");
  //ros::NodeHandle n;

  //ekumen_technical_exercise::RosPackageTemplate rosPackageTemplate(nodeHandle);
  while(ros::ok()) {
  	
  	ros::spin();
  }
  return 0;
}
