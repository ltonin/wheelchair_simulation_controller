#ifndef WHEELCHAIR_GOAL_GENERATOR_CPP
#define WHEELCHAIR_GOAL_GENERATOR_CPP

#include "wheelchair_simulation_controller/GoalGenerator.hpp"

GoalGenerator::GoalGenerator(void) : p_nh_("~") {
	this->goal_topic_ = "/current_goal";
	this->odom_topic_ = "/odom";
	this->auto_goal_  = true;
	this->reset_goal_distance_ = 1.5f;
}

GoalGenerator::~GoalGenerator(void) {}

bool GoalGenerator::configure(void) {


	XmlRpc::XmlRpcValue poses;
	this->p_nh_.getParam("goals", poses);

	this->initialize_goals(poses);
	this->set_current_goal(0);

	this->pub_ = this->nh_.advertise<geometry_msgs::PoseStamped>(this->goal_topic_, 1);
	this->sub_ = this->nh_.subscribe(this->odom_topic_, 1, &GoalGenerator::on_received_odom, this);
	this->srv_next_goal_ = this->p_nh_.advertiseService("next_goal", &GoalGenerator::on_request_next, this);
	this->srv_prev_goal_ = this->p_nh_.advertiseService("previous_goal", &GoalGenerator::on_request_prev, this);

	return true;
}

bool GoalGenerator::run(void) {

	float radius, gx, gy, ox, oy;

	ros::Rate r(100);

	while(ros::ok()) {

		this->current_goal_.header.frame_id = "odom";
		this->current_goal_.header.stamp = ros::Time::now();

		this->pub_.publish(this->current_goal_);

		if (this->auto_goal_ == true) {

			ox = this->odom_.pose.pose.position.x;
			oy = this->odom_.pose.pose.position.y;
			gx = this->current_goal_.pose.position.x;
			gy = this->current_goal_.pose.position.y;

			radius = std::sqrt( std::pow( (gx - ox), 2) + std::pow( (gy - oy), 2) );
			if (radius <= this->reset_goal_distance_) {
				if(this->set_next_goal() == true) {
					ROS_INFO("Next goal has been set: #%d", this->counter_+1);
				}
			}

		}


		ros::spinOnce();
		r.sleep();
	}

	return true;
}

void GoalGenerator::on_received_odom(const nav_msgs::Odometry& msg) {
	this->odom_ = msg;
}

bool GoalGenerator::on_request_next(std_srvs::Empty::Request& req,
									std_srvs::Empty::Response& res) {

	bool retcod = true;
	
	if(retcod = this->set_next_goal())
		ROS_INFO("Next goal has been set: #%d", this->counter_+1);


	return retcod;
}

bool GoalGenerator::on_request_prev(std_srvs::Empty::Request& req,
									std_srvs::Empty::Response& res) {

	bool retcod = true;

	if(retcod = this->set_previous_goal())
		ROS_INFO("Previous goal has been set: #%d", this->counter_+1);


	return retcod;
}


void GoalGenerator::initialize_goals(const XmlRpc::XmlRpcValue& poses) {

	geometry_msgs::PoseStamped msg;


	tf2::Quaternion quaternion;
	for(auto it = 0; it < poses.size(); ++it) {
		msg.pose.position.x = static_cast<double>(poses[it][0]);
		msg.pose.position.y = static_cast<double>(poses[it][1]);
		quaternion.setRPY(0, 0, static_cast<double>(poses[it][2]));
		quaternion.normalize();
		msg.pose.orientation = tf2::toMsg(quaternion);

		this->goals_.push_back(msg);
	}
}

bool GoalGenerator::set_next_goal(void) {
	
	bool retcod = true;

	if(this->counter_ == this->goals_.size() -1) {
		ROS_WARN("Current goal is already the last one");
		retcod = false;
	} else {
		this->counter_++;
		this->set_current_goal(this->counter_);
	}
	
	return retcod;
}

bool GoalGenerator::set_previous_goal(void) {

	bool retcod = true;

	if(this->counter_ == 0) {
		ROS_WARN("Current goal is already the first one");
		retcod = false;
	} else {
		this->counter_--;
		this->set_current_goal(this->counter_);
	}

	return retcod;
}

void GoalGenerator::set_current_goal(unsigned int goalId) {
	this->current_goal_ = this->goals_.at(this->counter_);
}

#endif
