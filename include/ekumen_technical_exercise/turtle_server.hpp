#pragma once

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <ekumen_technical_exercise/TurtleAction.h>
// Dynamic reconfigure includes
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory
#include <ekumen_technical_exercise/turtleMaxVelocityConfig.h>

#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_srvs/Empty.h>

#define PI 3.14159265

enum simState
{
	PAUSE,
	RESUME
};

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
	ros::ServiceServer srvPause;
	ros::ServiceServer srvResume;
	dynamic_reconfigure::Server<ekumen_technical_exercise::turtleMaxVelocityConfig> server;
	dynamic_reconfigure::Server<ekumen_technical_exercise::turtleMaxVelocityConfig>::CallbackType cb;
	double maxVel;
	double angVel;
	double tolerance;
	double angTol;
	simState state;
public:
	// Constructor & destructor
	TurtleServer();
	TurtleServer(std::string);
	// Callback
	void executeCb(const ekumen_technical_exercise::TurtleGoalConstPtr&);
	void turtlePoseCb(const turtlesim::Pose::ConstPtr&);
	bool resumeCb(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool pauseCb(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	void dynamicReconfigureCb(ekumen_technical_exercise::turtleMaxVelocityConfig&, double_t);
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
  tolerance(0.3),
  state(RESUME),
  cb(boost::bind(&TurtleServer::dynamicReconfigureCb, this, _1, _2)),
  maxVel(1.0),
  angVel(1.5),
  angTol(0.001) {

	as.start(); // auto_start = false

	ROS_INFO("\nAction Server started\n");

	// Topics
	poseSub = nh.subscribe("/turtle1/pose", 1, &TurtleServer::turtlePoseCb, this);
	twistPub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1);

	// Services
	srvResume = nh.advertiseService("/resume", &TurtleServer::resumeCb, this);
	srvPause = nh.advertiseService("/pause", &TurtleServer::pauseCb, this);

	// Dynamic reconfigure
	server.setCallback(cb);
}

double TurtleServer::getDistance(turtlesim::Pose &pose) {

	return std::sqrt(
		std::pow((pose.x - this->pose.x),2) 
		+ std::pow((pose.y - this->pose.y),2));
}

double TurtleServer::getAngleDiff(turtlesim::Pose& pose) {
	double angle =  std::atan2(
		(pose.y - this->pose.y), (pose.x - this->pose.x));
	// Converts from [-PI;PI] to [0;2*PI]
	if (angle < 0) angle = 2*PI + angle;
	// Set maximum value
	if (angle > 2*PI) angle = 2*PI;

	return angle;
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

bool TurtleServer::resumeCb(
	std_srvs::Empty::Request &req, 
	std_srvs::Empty::Response &res) {
	
	state = RESUME;
	return true;
}
	
bool TurtleServer::pauseCb(
	std_srvs::Empty::Request &req, 
	std_srvs::Empty::Response &res) {
	
	state = PAUSE;

	geometry_msgs::Twist velMsg;
	velMsg.linear.x = 0;
	velMsg.linear.y = 0;
	velMsg.linear.z = 0;
	velMsg.angular.x = 0;
	velMsg.angular.y = 0;
	velMsg.angular.z = 0;
	twistPub.publish(velMsg);

	return true;
}


void TurtleServer::dynamicReconfigureCb(
	ekumen_technical_exercise::turtleMaxVelocityConfig &config, 
	double_t level) 	// level parameter in cfg file
{	
	this->maxVel = config.max_vel;
	this->angVel = config.ang_vel;

	ROS_INFO("Reconfigure Request: [%f, %f]", config.max_vel, config.ang_vel);
}