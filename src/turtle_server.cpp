
#include "ekumen_technical_exercise/turtle_server.hpp"


double TurtleServer::goToGoal(turtlesim::Pose pose, double min_progress) {

	ros::topic::waitForMessage<turtlesim::Pose>("/turtle1/pose");

	ROS_INFO("\nTurtle going from [%f,%f] to [%f,%f]\n",
		this->pose.x,this->pose.y,pose.x,pose.y);

	geometry_msgs::Twist velMsg;
	velMsg.linear.x = 0;
	velMsg.linear.y = 0;
	velMsg.linear.z = 0;
	velMsg.angular.x = 0;
	velMsg.angular.y = 0;
	velMsg.angular.z = 0;

	double targetAngle = TurtleServer::getAngleDiff(pose);
	//ROS_INFO("[targetAngle] : %f\n", targetAngle);
	//ROS_INFO("[theta] : %f\n", this->pose.theta);

	while( fabsf(targetAngle - this->pose.theta) > 0.017) {

		velMsg.angular.z = 0.4;//this->K_ang * angleDiff;
		twistPub.publish(velMsg);
		//angleDiff = TurtleServer::getAngleDiff(pose);
		//ROS_INFO("\n[theta] : %f\n", this->pose.theta);

	}

	velMsg.angular.z = 0;
	twistPub.publish(velMsg);
	double maxDistance = TurtleServer::getDistance(pose);

	while(TurtleServer::getDistance(pose) >= this->tolerance) {

		// Implements a proportional controller
		// Linear velocity
		velMsg.linear.x = this->K_dist * TurtleServer::getDistance(pose);
		
		// Publish velocity
		twistPub.publish(velMsg);

		feedback.progress = (min_progress - TurtleServer::getDistance(pose) / maxDistance)*100;
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

	ROS_INFO("> Publishing result...\n");
	// Publish the result if the goal wasn't preempted
	result.result = success;
	if (success) {
		feedback.progress = 100.00;
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
