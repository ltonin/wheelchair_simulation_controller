#include <ros/ros.h>
#include "wheelchair_simulation_controller/CommandGenerator.hpp"

int main(int argc, char** argv) {

	// ros initialization
	ros::init(argc, argv, "commandgenerator");

	CommandGenerator generator;

	if(generator.configure() == false)
		ROS_ERROR("Command generator cannot be configured");

	if(generator.run() == false)
		ROS_ERROR("Command generator interrupted while running");

	ros::shutdown();
	return 0;

	return 0;
}
