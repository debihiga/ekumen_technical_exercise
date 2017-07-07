
#include "ekumen_technical_exercise/turtle_server.hpp"


double TurtleServer::goToGoal(turtlesim::Pose pose, double topLimit) {

	ros::topic::waitForMessage<turtlesim::Pose>("/turtle1/pose");

	if(! TurtleServer::orientTurtle(pose)) return false;
	if(! TurtleServer::moveToGoal(pose, topLimit)) return false;

	TurtleServer::stopTurtle();

	return true;
}

void TurtleServer::executeCb(const ekumen_technical_exercise::TurtleGoalConstPtr &goal) {
	
	// If the server has been killed, don't process
	if(!as.isActive()||as.isPreemptRequested()) return;

	// Run the processing at 100Hz
	ros::Rate rate(100);

	// Setup some local variables
	bool success = true;	
	
	uint8_t path_len = goal->path_length;

	if (path_len) {

		uint8_t i = 0;

		feedback.progress = 0.00;
		feedback.state = "RUNNING";
		as.publishFeedback(feedback);

		for(std::vector<turtlesim::Pose>::const_iterator it = goal->pose.begin(); it != goal->pose.end(); ++it)
		{
			success = TurtleServer::goToGoal(*it, (double)1/path_len);	
			if (!i) {
				std_srvs::Empty::Request req;
				std_srvs::Empty::Response res;
				if(!srvClear.call(req,res)) {
					ROS_ERROR("\nFailed to call service \"clear\"");
				}
			}
			i++;

			if (!success)	break;
		}

	} else {
		ROS_ERROR("\nPath received with no length\n");
	}

	// Publish the result if the goal wasn't preempted
	result.result = success;
	if (success) {
		feedback.progress = 1.00;
		feedback.state = "PATH COMPLETED";
		as.publishFeedback(feedback);
		as.setSucceeded(result);
	} else {
		as.setAborted(result, "Failed to reach the goal");
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtle_server_node");

  TurtleServer server(ros::this_node::getName());

  ros::spin();
  
  return 0;
}
