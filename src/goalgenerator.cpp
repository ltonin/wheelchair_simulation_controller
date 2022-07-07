#include <ros/ros.h>
#include "wheelchair_simulation_controller/GoalGenerator.hpp"

int main(int argc, char** argv) {

	// ros initialization
	ros::init(argc, argv, "goalgenerator");

	GoalGenerator generator;

	if(generator.configure() == false)
		ROS_ERROR("Goal generator cannot be configured");

	if(generator.run() == false)
		ROS_ERROR("Goal generator interrupted while running");

	ros::shutdown();
	return 0;

	return 0;
}
