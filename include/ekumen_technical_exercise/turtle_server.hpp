#pragma once

#include <ros/ros.h>
// Actionlib include files
#include <actionlib/server/simple_action_server.h>
#include <ekumen_technical_exercise/TurtleAction.h>
// Dynamic reconfigure includes
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory
#include <ekumen_technical_exercise/turtleMaxVelocityConfig.h>

#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
// rosservice call /clear "{}"
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
	// Callback
	void executeCb(const ekumen_technical_exercise::TurtleGoalConstPtr&);
	void turtlePoseCb(const turtlesim::Pose::ConstPtr&);
	bool resumeCb(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool pauseCb(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	void dynamicReconfigureCb(ekumen_technical_exercise::turtleMaxVelocityConfig&, double_t);
	// Variables
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
	ros::ServiceClient srvClear;
	dynamic_reconfigure::Server<ekumen_technical_exercise::turtleMaxVelocityConfig> server;
	dynamic_reconfigure::Server<ekumen_technical_exercise::turtleMaxVelocityConfig>::CallbackType cb;
	double maxVel;
	double angVel;
	double distTol;
	double angTol;
	double prevProgress;
	simState state;
public:
	// Constructor & destructor
	TurtleServer();
	TurtleServer(std::string);
	// Functions
	double getDistance(turtlesim::Pose&);
	double getAngleDiff(turtlesim::Pose&);
	double goToGoal(turtlesim::Pose, double);
	double wrapAngle(double);
	void emptyVelMsg(geometry_msgs::Twist&);
	bool orientTurtle(turtlesim::Pose&);
	bool moveToGoal(turtlesim::Pose&, double&);
	void stopTurtle(void);
	void updateProgress(turtlesim::Pose&, double&, double&);
};

/*
 * Constructors
 */

TurtleServer::TurtleServer()
: TurtleServer::TurtleServer("turtle_server") {}

TurtleServer::TurtleServer(std::string name)
: as(nh, 							// node handle
	"turtle_action", 				// action server name
	boost::bind(
		&TurtleServer::executeCb, 	// is executed when a goal is received
		this, 
		_1), 
	false),							// auto start
  actionName(name),
  distTol(5e-2),
  angTol(1e-4),
  state(RESUME),
  cb(boost::bind(&TurtleServer::dynamicReconfigureCb, this, _1, _2)),
  maxVel(1.0),
  angVel(1.5),
  prevProgress(0.0) {

  	srvClear = nh.serviceClient<std_srvs::Empty>("clear");

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
	// Euclidean distance
	return std::sqrt(
		std::pow((pose.x - this->pose.x),2) 
		+ std::pow((pose.y - this->pose.y),2));
}

double TurtleServer::getAngleDiff(turtlesim::Pose& pose) {
	double angle =  std::atan2(
		(pose.y - this->pose.y), (pose.x - this->pose.x));
	// Converts from [-PI;PI] to [0;2*PI]
	if (angle < 0) angle += 2*PI;
	// Set maximum value
	if (angle > 2*PI) angle = 2*PI;

	return fabs(angle - this->pose.theta);
}

double TurtleServer::wrapAngle(double angle) {
	// Wrap angle between [0;2PI]
	angle = std::fmod(angle, 2*PI); 
 	if (angle < 0)	angle += 2*PI;
 	return angle;
}

void TurtleServer::turtlePoseCb(const turtlesim::Pose::ConstPtr& msg) {
	// Subscribe to the position of the robot and store it into variables
	this->pose.x = msg->x;
	this->pose.y = msg->y;
	this->pose.theta = TurtleServer::wrapAngle(msg->theta);
}

bool TurtleServer::resumeCb(
	// Continue executing the simulation
	std_srvs::Empty::Request &req, 
	std_srvs::Empty::Response &res) {
	
	state = RESUME;
	return true;
}
	
bool TurtleServer::pauseCb(
	// Pause the execution
	std_srvs::Empty::Request &req, 
	std_srvs::Empty::Response &res) {
	
	state = PAUSE;

	TurtleServer::stopTurtle();

	return true;
}


void TurtleServer::dynamicReconfigureCb(
	ekumen_technical_exercise::turtleMaxVelocityConfig &config, 
	double_t level) 	// level parameter in cfg file
{	
	// Change the maximum allowed velocities
	this->maxVel = config.max_vel;
	this->angVel = config.ang_vel;

	ROS_INFO("Reconfigure Request: [%f, %f]", config.max_vel, config.ang_vel);
}

void TurtleServer::emptyVelMsg(geometry_msgs::Twist& msg) {
	// Creates a Twist message with all values in zero
	msg.linear.x = 0;
	msg.linear.y = 0;
	msg.linear.z = 0;
	msg.angular.x = 0;
	msg.angular.y = 0;
	msg.angular.z = 0;
}

bool TurtleServer::orientTurtle(turtlesim::Pose &_pose) {
	// Rotate the robot pointing to the goal
	geometry_msgs::Twist velMsg;
	TurtleServer::emptyVelMsg(velMsg);
	// How much angle difference is needed?
	double angleDiff = TurtleServer::getAngleDiff(_pose);

	ROS_INFO("\nTurtle going from [%f,%f] to [%f,%f]\n",
		this->pose.x, this->pose.y, _pose.x, _pose.y);

	while( angleDiff > angTol) {
		// Proportional controller IF THE SIMULATION IS NOT PAUSED
		state == RESUME ? velMsg.angular.z = this->angVel * angleDiff : velMsg.angular.z = 0;
		
		twistPub.publish(velMsg);

		// Check for ROS kill
		if(!ros::ok() || !nh.ok()) {
			ROS_INFO("%s Shutting Down", actionName.c_str());
			return false;
		}

		angleDiff = TurtleServer::getAngleDiff(_pose);
	}
	// Stop the robot
	TurtleServer::stopTurtle();

	return true;
}


bool TurtleServer::moveToGoal(turtlesim::Pose &pose, double &topLimit) {
	geometry_msgs::Twist velMsg;
	TurtleServer::emptyVelMsg(velMsg);

	double maxDistance = TurtleServer::getDistance(pose);

	while(TurtleServer::getDistance(pose) >= this->distTol) {

		// Implements a proportional controller
		// Linear velocity
		state == RESUME ? velMsg.linear.x = this->maxVel * TurtleServer::getDistance(pose) : velMsg.linear.x = 0;
		
		// Publish velocity
		twistPub.publish(velMsg);

		TurtleServer::updateProgress(pose, topLimit, maxDistance);

		// Check for ROS kill
		if(!ros::ok() || !nh.ok()) {
			ROS_INFO("%s Shutting Down", actionName.c_str());
			return false;
		}
		// To reduce the publishing rate and problems with the web page
		ros::Duration(0.1).sleep();
	}
	// Update this variable is necessary to show the correct
	// porcentual progress on screen
	prevProgress = feedback.progress;
	return true;
}


void TurtleServer::stopTurtle() {
	// Stop the turtlesim	
	geometry_msgs::Twist velMsg;
	TurtleServer::emptyVelMsg(velMsg);
	twistPub.publish(velMsg);
}


void TurtleServer::updateProgress(turtlesim::Pose &pose, double &topLimit, double &maxDistance) {
	// Calculate the progress of the robot respect to the complete path
	feedback.progress = prevProgress + topLimit * (1 - (TurtleServer::getDistance(pose) / maxDistance));
	// Send the current status of the simulation in order to be shown on the screen
	state == RESUME ? feedback.state = "RUNNING" : feedback.state = "PAUSE";
	as.publishFeedback(feedback);
}