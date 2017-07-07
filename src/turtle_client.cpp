
#include "ekumen_technical_exercise/turtle_client.hpp"


void TurtleClient::doneCb(
	const actionlib::SimpleClientGoalState& state, 
	const ekumen_technical_exercise::TurtleResultConstPtr& result) {
	// Called once when the goal completes
	
	//ROS_INFO("Finished in state [%s]", state.toString().c_str());
	//ROS_INFO("Result: %d", result->ok);
	//ROS_INFO("> Reached goal\n");
}

void TurtleClient::activeCb() {
// Called once when the goal becomes active
	//ROS_INFO("> Performing path\n");
}

void TurtleClient::feedbackCb(const ekumen_technical_exercise::TurtleFeedbackConstPtr& feedback) {
	//ROS_INFO("progress: %.2f\n", feedback->progress);
	// Prints the progress bar
	// https://stackoverflow.com/a/14539953
	if(feedback->progress <= 1.0) {
		int barWidth = 70;

		std::cout << "[";
		int pos = barWidth * feedback->progress;
		for (int i = 0; i < barWidth; ++i) {
			if (i < pos) std::cout << "="; 
			else if (i == pos) std::cout << ">";
			else std::cout << " ";
		}
		std::cout << "] " << int(feedback->progress * 100.00) << " %\r";
		std::cout.flush();
	} else {
		std::cout << std::endl << std::endl;
	}
}

void TurtleClient::setGoal() {
	
	// send a goal to the action
	ekumen_technical_exercise::TurtleGoal goal;
	turtlesim::Pose _pose;
	
	double numNodes;
	if(!nh.getParam("/turtle_client/num_nodes", numNodes)) {
		ROS_ERROR("> Error retrieving path parameters : %f", numNodes);
		return;
	} else {
		//ROS_INFO("> Nodes found : %f\n", numNodes);
	}
	
	if(numNodes > 0) {
		double yamlData;
		// The number of nodes is, at least, one
		for (uint8_t i = 0; i < (uint8_t)numNodes; i++) {
			nh.param<double>("/turtle_client/node" + std::to_string(i) + "/x", yamlData, 5.5444);
			_pose.x = yamlData;
			nh.param<double>("/turtle_client/node" + std::to_string(i) + "/y", yamlData, 5.5444);
			_pose.y = yamlData;
			nh.param<double>("/turtle_client/node" + std::to_string(i) + "/theta", yamlData, 0.0);
			_pose.theta = yamlData;

			goal.pose.push_back(_pose);
		}
	}
	
	goal.path_length = goal.pose.size();
	
	ac.sendGoal(
		goal,
		boost::bind(&TurtleClient::doneCb, this, _1, _2),
		boost::bind(&TurtleClient::activeCb, this),
		boost::bind(&TurtleClient::feedbackCb, this, _1)
	);
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
