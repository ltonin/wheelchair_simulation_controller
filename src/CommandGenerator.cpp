#ifndef WHEELCHAIR_COMMAND_GENERATOR_CPP
#define WHEELCHAIR_COMMAND_GENERATOR_CPP

#include "wheelchair_simulation_controller/CommandGenerator.hpp"

CommandGenerator::CommandGenerator(void) : p_nh_("~"), rnd_gen_((std::random_device())()) {
}

CommandGenerator::~CommandGenerator(void) {}

bool CommandGenerator::configure(void) {


	// Get user parameters from server
	XmlRpc::XmlRpcValue p_user;
	if(this->p_nh_.getParam("user", p_user) == true) {
		this->probability_inc_ = static_cast<double>(p_user["probability_inc"]);
		this->probability_acc_ = static_cast<double>(p_user["probability_acc"]);
		this->min_delivery_time_ = static_cast<double>(p_user["min_delivery_time"]);
		this->max_delivery_time_ = static_cast<double>(p_user["max_delivery_time"]);
		this->refractory_period_ = static_cast<double>(p_user["refractory_period"]);
	} else {
		ROS_WARN("Cannot load user parameters from server. Using default values");
		this->probability_inc_ = 0.70f;
		this->probability_acc_ = 0.80f;
		this->min_delivery_time_ = 2.0f;
		this->max_delivery_time_ = 4.0f;
		this->refractory_period_ = 5.0f;
	}
	
	// Get target parameters from server
	XmlRpc::XmlRpcValue p_target;
	if(this->p_nh_.getParam("target", p_target) == true) {
		this->ttopic_ 		= static_cast<std::string>(p_target["topic"]);
		this->front_sector_ = static_cast<double>(p_target["front_sector"]);
	} else {
		ROS_WARN("Cannot load target parameters from server. Using default values");
		this->ttopic_ 		= "/current_goal";
		this->front_sector_ = 0.5235f;
	}
	
	// Get command parameters from server
	XmlRpc::XmlRpcValue p_command;
	if(this->p_nh_.getParam("command", p_command) == true) {
		this->frame_id_ 		  	= static_cast<std::string>(p_command["frame_id"]);
		this->ctopic_ 			  	= static_cast<std::string>(p_command["topic"]);
		this->command_angle_left  	= static_cast<double>(p_command["angle_left"]);
		this->command_angle_right	= static_cast<double>(p_command["angle_right"]);
		this->command_angle_forward = static_cast<double>(p_command["angle_forward"]);
		this->command_distance 		= static_cast<double>(p_command["distance"]);
	} else {
		ROS_WARN("Cannot load command parameters from server. Using default values");
		this->frame_id_ 		  	= "hokuyo_link"; 
		this->ctopic_ 			  	= "/bci_command";
		this->command_angle_left  	= 2.356f;
		this->command_angle_right	= 0.785f;
		this->command_angle_forward = 1.570f;
		this->command_distance 		= 1.0f;
	}



	// this->ttopic_   	 = "/current_goal";
	//this->ctopic_   	 = "/bci_command";
	//this->frame_id_ 	 = "hokuyo_link";
	this->first_target_  = false;
	//this->front_sector_ 	 = 0.5235f; // 30 deg 
	//this->min_delivery_time_ = 2.0f;
	//this->max_delivery_time_ = 4.0f;
	//this->refractory_period_ = 5.0f;

	//this->probability_inc_ = 0.70f; 	// Probability to stay in INC
	//this->probability_acc_ = 0.85f;		// Probability to fire a correct command

	this->sub_target_ = this->nh_.subscribe(this->ttopic_, 1, &CommandGenerator::on_received_target, this);

	this->pub_cmd_ = this->nh_.advertise<geometry_msgs::PointStamped>(this->ctopic_, 1);

	this->rnd_time_dist_ = std::uniform_real_distribution<>(this->min_delivery_time_ + this->refractory_period_, this->refractory_period_ + this->max_delivery_time_);
	this->rnd_real_dist_ = std::uniform_real_distribution<>(0.0f, 1.0f);

	this->init_random_timer();
	this->cmd_timer_.stop();
	return true;
}

bool CommandGenerator::run(void) {
	ros::Rate r(100);

	while(ros::ok()) {


		if (first_target_ == true) {
			this->set_target_position();
			this->cmd_timer_.start();
		}


		ros::spinOnce();
		r.sleep();
	}

	return true;
}

void CommandGenerator::init_random_timer(void) {

	float rnddur;

	rnddur = this->rnd_time_dist_(this->rnd_gen_);

	this->cmd_timer_ = this->nh_.createTimer(ros::Duration(rnddur), &CommandGenerator::on_command_firing, this);

	printf("Initialized timer with duration: %f s\n", rnddur);
}

void CommandGenerator::on_received_target(const geometry_msgs::PoseStamped& msg) {

	this->first_target_ = true;

	this->target_ = msg;

}
		
void CommandGenerator::on_command_firing(const ros::TimerEvent& event) {


	bool is_fired = false;
	std::string cmd_label;

	switch(this->target_pos_) {
		// If the target position is onFront, then use the INC probability to check
		// if a random command is fired
		case TargetPosition::onFront:
			if (this->get_random_bool(this->probability_inc_) == false) {
				ROS_INFO("Random INC command is fired");
				this->set_random_command();
				is_fired = true;
			}
			break;
		// If the target position is on LEFT | RIGHT, then use the ACC probability
		// to check if a correct or wrong command is fired
		case TargetPosition::onRight:
		case TargetPosition::onLeft:
			if(this->get_random_bool(this->probability_acc_) == false) {
				cmd_label = this->target_pos_ == TargetPosition::onLeft ? "Right" : "Left";
				ROS_INFO("Wrong command is fired: %s", cmd_label.c_str());
				this->set_wrong_command();
			} else {
				cmd_label = this->target_pos_ == TargetPosition::onLeft ? "Left" : "Right";
				ROS_INFO("Correct command is fired: %s", cmd_label.c_str());
				this->set_correct_command();
			}
			is_fired = true;
			break;
		default:
			break;
	}

	if(is_fired == true) {
		this->pub_cmd_.publish(this->user_cmd_);
	}

	this->init_random_timer();

}

void CommandGenerator::set_target_position(void) {

	std::string tframe_id;
 	geometry_msgs::PoseStamped tf_target;
	float angle = 0.0f;

	tframe_id = this->target_.header.frame_id;

	try {
		this->listener_.waitForTransform(this->frame_id_, tframe_id, ros::Time::now(), ros::Duration(1.0));
		this->listener_.transformPose(this->frame_id_, this->target_, tf_target);
	}  catch (tf::TransformException &ex) {
		ROS_ERROR("%s", ex.what());
	}

	angle = std::atan2(tf_target.pose.position.y, tf_target.pose.position.x);
	//printf("Target angle with respect to %s: %f [deg]\n", this->frame_id_.c_str(), angle);

	this->target_pos_ = this->get_target_position(angle, this->front_sector_);

}

void CommandGenerator::set_correct_command(void) {

	float angle;
	float distance = this->command_distance;

	switch(this->target_pos_) {
		case TargetPosition::onLeft:
			angle = this->command_angle_left;
			this->user_cmd_ = this->get_command_message(angle, distance);
			break;
		case TargetPosition::onRight:
			angle = this->command_angle_right;
			this->user_cmd_ = this->get_command_message(angle, distance);
			break;
		default:
			break;
	}
}

void CommandGenerator::set_wrong_command(void) {
	float angle;
	float distance = this->command_distance;

	switch(this->target_pos_) {
		case TargetPosition::onLeft:
			angle = this->command_angle_right;
			this->user_cmd_ = this->get_command_message(angle, distance);
			break;
		case TargetPosition::onRight:
			angle = this->command_angle_left;
			this->user_cmd_ = this->get_command_message(angle, distance);
			break;
		default:
			break;
	}

}

void CommandGenerator::set_random_command(float probability) {

	if(this->get_random_bool(probability) == true) {
		this->user_cmd_ = this->get_command_message(this->command_angle_left, this->command_distance);
	} else {
		this->user_cmd_ = this->get_command_message(this->command_angle_right, this->command_distance);
	}
}

geometry_msgs::PointStamped CommandGenerator::get_command_message(float angle, float distance) {

	geometry_msgs::PointStamped msg;

	msg.header.frame_id = this->frame_id_;
	msg.header.stamp 	= ros::Time::now();
	msg.point.x 		= distance*sin(angle);
	msg.point.y 		= -distance*cos(angle);
	msg.point.z 		= 0.0f;

	return msg;

}

TargetPosition CommandGenerator::get_target_position(float angle, float front_sector) {

	float llimit, rlimit;
	TargetPosition position;

	llimit = -front_sector/2.0f;
	rlimit = front_sector/2.0f;

	if ( (angle >= llimit) && (angle <= rlimit) ) {
		position = TargetPosition::onFront;
	} else if (angle < llimit ) {
		position = TargetPosition::onRight;
	} else if (angle > rlimit ) {
		position = TargetPosition::onLeft;
	} else {
		position = TargetPosition::None;
	}

	return position;

}

/*bool CommandGenerator::set_command_message(void) {

	float angle;
	float distance = this->command_distance;
	bool retcod = true;

	switch(this->correct_cmd_) {
		case UserCommand::LEFT:
			angle = this->command_angle_left;
			break;
		case UserCommand::RIGHT:
			angle = this->command_angle_right;
			break;
		default:
			retcod = false;
			break;
	}

	this->user_cmd_.header.frame_id = this->frame_id_;
	this->user_cmd_.header.stamp = ros::Time::now();
	this->user_cmd_.point.x = distance*sin(angle);
	this->user_cmd_.point.y = -distance*cos(angle);
	this->user_cmd_.point.z = 0;

	return retcod;

}*/

float CommandGenerator::rad2deg(float rad) {
	return (rad*180.0f)/pi;
}

float CommandGenerator::deg2rad(float deg) {
	return (deg*pi)/180.0f;
}

bool CommandGenerator::get_random_bool(float probability) {

	float rnd;
	bool retcod = false;
	rnd = this->rnd_real_dist_(this->rnd_gen_);

	if (rnd <= probability)
		retcod = true;

	return retcod;
}




#endif
