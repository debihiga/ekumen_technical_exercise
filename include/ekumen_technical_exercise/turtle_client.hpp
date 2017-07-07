#pragma once

#include <ros/ros.h>
#include <ekumen_technical_exercise/TurtleAction.h>
#include <actionlib/client/simple_action_client.h>

#include <turtlesim/Pose.h>


class TurtleClient
{
public:
	TurtleClient();
	TurtleClient(std::string);
	// Functions
	void setGoal();
private:
	ros::NodeHandle nh;
	actionlib::SimpleActionClient<ekumen_technical_exercise::TurtleAction> ac;
	std::string actionName;
	// Callback
	void doneCb(const actionlib::SimpleClientGoalState&, const ekumen_technical_exercise::TurtleResultConstPtr&);
	void activeCb();
	void feedbackCb(const ekumen_technical_exercise::TurtleFeedbackConstPtr&);
};

TurtleClient::TurtleClient()
: TurtleClient::TurtleClient("turtle_client_test") {}

TurtleClient::TurtleClient(std::string name)
: nh("~"),
  actionName(name),
  ac("turtle_action", true) {

  	// Get connection to a server
	ROS_INFO("%s Waiting For Server...\n", actionName.c_str());

	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("%s started, sending goal.\n", actionName.c_str());
}