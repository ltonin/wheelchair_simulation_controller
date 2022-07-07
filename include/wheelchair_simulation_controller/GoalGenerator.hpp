#ifndef WHEELCHAIR_GOAL_GENERATOR_HPP
#define WHEELCHAIR_GOAL_GENERATOR_HPP

#include <vector>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class GoalGenerator {
	public:
		GoalGenerator(void);
		~GoalGenerator(void);
		bool configure(void);
		bool run(void);


	private:
		bool on_request_next(std_srvs::Empty::Request& req,
							 std_srvs::Empty::Response& res);
		
		bool on_request_prev(std_srvs::Empty::Request& req,
							 std_srvs::Empty::Response& res);
		
		void on_received_odom(const nav_msgs::Odometry& msg);

		void initialize_goals(const XmlRpc::XmlRpcValue& poses);
		bool set_next_goal(void);
		bool set_previous_goal(void);
		void set_current_goal(unsigned int goalId);

	private:
		ros::NodeHandle		nh_;
		ros::NodeHandle 	p_nh_;

		ros::ServiceServer	srv_next_goal_;
		ros::ServiceServer	srv_prev_goal_;
		ros::ServiceServer	srv_set_goal_;
		ros::Publisher 		pub_;
		ros::Subscriber		sub_;
		std::string			goal_topic_;
		std::string			odom_topic_;
		bool auto_goal_;

		unsigned int 		counter_ = 0;
		geometry_msgs::PoseStamped 	current_goal_;
		nav_msgs::Odometry			odom_;
		std::vector<geometry_msgs::PoseStamped> goals_;
		float reset_goal_distance_;

};

#endif
