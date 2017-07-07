
#include "ekumen_technical_exercise/turtle_server.hpp"


double TurtleServer::goToGoal(turtlesim::Pose pose, double topLimit) {

	ros::topic::waitForMessage<turtlesim::Pose>("/turtle1/pose");

	geometry_msgs::Twist velMsg;
	velMsg.linear.x = 0;
	velMsg.linear.y = 0;
	velMsg.linear.z = 0;
	velMsg.angular.x = 0;
	velMsg.angular.y = 0;
	velMsg.angular.z = 0;

	double targetAngle = TurtleServer::getAngleDiff(pose);

	ROS_INFO("\nTurtle going from [%f,%f,%f] to [%f,%f,%f]\n",
		this->pose.x, this->pose.y, this->pose.theta,
		pose.x, pose.y, targetAngle);

	while( fabs(targetAngle - this->pose.theta) > angTol) {

		state == RESUME ? velMsg.angular.z = this->angVel * (fabsf(targetAngle - this->pose.theta)) : velMsg.angular.z = 0;
		
		twistPub.publish(velMsg);
	}

	velMsg.angular.z = 0;
	twistPub.publish(velMsg);
	double maxDistance = TurtleServer::getDistance(pose);

	while(TurtleServer::getDistance(pose) >= this->tolerance) {

		// Implements a proportional controller
		// Linear velocity
		state == RESUME ? velMsg.linear.x = this->maxVel * TurtleServer::getDistance(pose) : velMsg.linear.x = 0;
		
		// Publish velocity
		twistPub.publish(velMsg);

		feedback.progress = (topLimit - (TurtleServer::getDistance(pose) * topLimit / maxDistance));
		state == RESUME ? feedback.state = "RUNNING" : feedback.state = "PAUSE";
		as.publishFeedback(feedback);

		// Check for ROS kill
		if(!ros::ok()) {
			ROS_INFO("%s Shutting Down", actionName.c_str());
			return false;
		}

		ros::Duration(0.1).sleep();
	}

	// Stop the turtlesim
	velMsg.linear.x = 0;
	velMsg.angular.z = 0;
	twistPub.publish(velMsg);

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
	ROS_INFO("path length : %d", path_len);

	if (path_len) {

		uint8_t i = 0;

		for(std::vector<turtlesim::Pose>::const_iterator it = goal->pose.begin(); it != goal->pose.end(); ++it)
		{
			success = TurtleServer::goToGoal(*it, (double)(i+1)/path_len);
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
