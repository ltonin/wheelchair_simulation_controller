#ifndef WHEELCHAIR_COMMAND_GENERATOR_CPP
#define WHEELCHAIR_COMMAND_GENERATOR_CPP

#include "wheelchair_simulation_controller/CommandGenerator.hpp"

CommandGenerator::CommandGenerator(void) : p_nh_("~") {
}

CommandGenerator::~CommandGenerator(void) {}

bool CommandGenerator::configure(void) {


	this->ttopic_   = "/current_goal";
	this->frame_id_ = "hokuyo_link";
	this->first_target_ = false;

	this->sub_target_ = this->nh_.subscribe(this->ttopic_, 1, &CommandGenerator::on_received_target, this);


	return true;
}

bool CommandGenerator::run(void) {
	ros::Rate r(100);

	while(ros::ok()) {


		if (first_target_ == true) {
			this->set_correct_command();
		}


		ros::spinOnce();
		r.sleep();
	}

	return true;
}


void CommandGenerator::on_received_target(const geometry_msgs::PoseStamped& msg) {

	this->first_target_ = true;

	this->target_ = msg;

}

void CommandGenerator::set_correct_command(void) {

	std::string tframe_id;
 	geometry_msgs::PoseStamped tf_target;

	tframe_id = this->target_.header.frame_id;

	try {
		this->listener_.waitForTransform(this->frame_id_, tframe_id, ros::Time::now(), ros::Duration(10.0));
		this->listener_.transformPose(this->frame_id_, this->target_, tf_target);
	}  catch (tf::TransformException &ex) {
		ROS_ERROR("%s", ex.what());
	}

	double pi = 3.14159265358979323846;
	float angle = std::atan2(tf_target.pose.position.y, tf_target.pose.position.x);
	printf("Target angle with respect to %s: %f [deg]\n", this->frame_id_.c_str(), (angle*180.0f)/pi);

}




#endif
