
#include "ekumen_technical_exercise/turtle_client.hpp"


void TurtleClient::doneCb(
	const actionlib::SimpleClientGoalState& state, 
	const ekumen_technical_exercise::TurtleResultConstPtr& result) {
// Called once when the goal completes
	
	//ROS_INFO("Finished in state [%s]", state.toString().c_str());
	//ROS_INFO("Result: %d", result->ok);
	ROS_INFO("doneCb()\n");
}

void TurtleClient::activeCb() {
// Called once when the goal becomes active
	ROS_INFO("Performing path\n");
}

void TurtleClient::feedbackCb(const ekumen_technical_exercise::TurtleFeedbackConstPtr& feedback) {
	ROS_INFO("progress: %f\n", feedback->progress);
}

void TurtleClient::setGoal() {

	// send a goal to the action
	std::vector<turtlesim::Pose> _pose_v;
	turtlesim::Pose _pose;
	/*
	_pose.x = 0;
	_pose.y = 0;
	_pose.theta = 0;
	goal.pose.push_back(_pose);
	*/
	_pose.x = 5.55;
	_pose.y = 8;
	_pose.theta = 0;
	goal.pose.push_back(_pose);	

	goal.path_length = goal.pose.size();

	ac.sendGoal(
		goal,
		boost::bind(&TurtleClient::doneCb, this, _1, _2),
		boost::bind(&TurtleClient::activeCb, this),
		boost::bind(&TurtleClient::feedbackCb, this, _1)
	);
	/*
	// wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout) {
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	} else
		ROS_INFO("Action did not finish before the time out.");
	*/
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtle_client_test_node");

  // Create the action client
  TurtleClient client(ros::this_node::getName());

  client.setGoal();

  ros::spin();

  //exit
  return 0;
}
