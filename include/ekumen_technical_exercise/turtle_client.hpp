#pragma once

#include <ros/ros.h>
#include <ekumen_technical_exercise/TurtleAction.h>
#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/Pose2D.h>


class TurtleClient
{
public:
	TurtleClient();
	TurtleClient(std::string);
	// Functions
	void setGoal();
private:
	actionlib::SimpleActionClient<ekumen_technical_exercise::TurtleAction> ac;
	std::string actionName;
	ekumen_technical_exercise::TurtleGoal goal;
	// Callback
	void doneCb(const actionlib::SimpleClientGoalState&, const ekumen_technical_exercise::TurtleResultConstPtr&);
	void activeCb();
	void feedbackCb(const ekumen_technical_exercise::TurtleFeedbackConstPtr&);
};

TurtleClient::TurtleClient()
: TurtleClient::TurtleClient("turtle_client_test") {}

TurtleClient::TurtleClient(std::string name)
: actionName(name),
  ac("turtle_client", true) {

  	// Get connection to a server
	ROS_INFO("%s Waiting For Server...", actionName.c_str());

	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("%s started, sending goal.", actionName.c_str());
}