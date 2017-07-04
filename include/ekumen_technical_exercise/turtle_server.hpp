#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ekumen_technical_exercise/TurtleAction.h>

//#include <geometry_msgs/Pose2D.h>


class TurtleServer
{
protected:
	ros::NodeHandle nh;
	actionlib::SimpleActionServer<ekumen_technical_exercise::TurtleAction> as;
	ekumen_technical_exercise::TurtleFeedback feedback;
	ekumen_technical_exercise::TurtleResult result;
	std::string actionName;
public:
	// Constructor & destructor
	TurtleServer();
	TurtleServer(std::string);
	// Callback
	void executeCb(const ekumen_technical_exercise::TurtleGoalConstPtr&);
};

TurtleServer::TurtleServer()
: TurtleServer::TurtleServer("turtle_server") {}

TurtleServer::TurtleServer(std::string name)
: as(nh, 							// node handle
	name, 							// action server name
	boost::bind(
		&TurtleServer::executeCb, 	// is executed when a goal is received
		this, 
		_1), 
	false),							// auto start
actionName(name) {

	as.start(); // auto_start = false

	ROS_INFO("\nAction Server started\n");
}
