#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ekumen_technical_exercise/TurtleAction.h>

#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#define PI 3.14159265

class TurtleServer
{
protected:
	ros::NodeHandle nh;
	actionlib::SimpleActionServer<ekumen_technical_exercise::TurtleAction> as;
	ekumen_technical_exercise::TurtleFeedback feedback;
	ekumen_technical_exercise::TurtleResult result;
	std::string actionName;
	turtlesim::Pose pose;
	ros::Subscriber poseSub;
	ros::Publisher twistPub;
	double tolerance;
	double K_dist;
	double K_ang;
public:
	// Constructor & destructor
	TurtleServer();
	TurtleServer(std::string);
	// Callback
	void executeCb(const ekumen_technical_exercise::TurtleGoalConstPtr&);
	void turtlePoseCb(const turtlesim::Pose::ConstPtr&);
	// Functions
	double getDistance(turtlesim::Pose&);
	double getAngleDiff(turtlesim::Pose&);
	double goToGoal(turtlesim::Pose, double);
	double wrapAngle(double);
};

TurtleServer::TurtleServer()
: TurtleServer::TurtleServer("turtle_server") {}

TurtleServer::TurtleServer(std::string name)
: as(nh, 							// node handle
	"turtle_action", 							// action server name
	boost::bind(
		&TurtleServer::executeCb, 	// is executed when a goal is received
		this, 
		_1), 
	false),							// auto start
actionName(name),
tolerance(0.05),
K_dist(1.25),
K_ang(4) {

	as.start(); // auto_start = false

	ROS_INFO("\nAction Server started\n");

	poseSub = nh.subscribe("/turtle1/pose", 1, &TurtleServer::turtlePoseCb, this);
	twistPub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1);
}

double TurtleServer::getDistance(turtlesim::Pose &pose) {

	return std::sqrt(
		std::pow((pose.x - this->pose.x),2) 
		+ std::pow((pose.y - this->pose.y),2));
}

double TurtleServer::getAngleDiff(turtlesim::Pose& pose) {
	return std::atan2(
		(pose.y - this->pose.y),
		(pose.x - this->pose.x))
		- this->pose.theta;
}

double TurtleServer::wrapAngle(double angle) {
	angle = std::fmod(angle, 2*PI); 
 	if (angle < 0)	angle += 2*PI;
 	return angle;
}

void TurtleServer::turtlePoseCb(const turtlesim::Pose::ConstPtr& msg) {

	this->pose.x = msg->x;
	this->pose.y = msg->y;
	this->pose.theta = TurtleServer::wrapAngle(msg->theta);
}
