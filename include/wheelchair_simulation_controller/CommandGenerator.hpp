#ifndef WHEELCHAIR_COMMAND_GENERATOR_HPP
#define WHEELCHAIR_COMMAND_GENERATOR_HPP

#include <random>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>


enum class TargetPosition {None, onLeft, onFront, onRight};

class CommandGenerator {

	public:
		CommandGenerator(void);
		~CommandGenerator(void);
		bool configure(void);
		bool run(void);

	private:
		void on_received_target(const geometry_msgs::PoseStamped& msg);
		void on_command_firing(const ros::TimerEvent& event);

		void init_random_timer(void);
		
		void set_correct_command(void);
		void set_wrong_command(void);
		void set_random_command(float probability = 0.5f);
		geometry_msgs::PointStamped get_command_message(float angle, float distance);


		//bool set_command_message(void);
		float rad2deg(float rad);
		float deg2rad(float rad);
		bool get_random_bool(float probability);

		void set_target_position(void);
		TargetPosition get_target_position(float angle, float front_sector);

	private:
		ros::NodeHandle			nh_;
		ros::NodeHandle			p_nh_;
		tf::TransformListener	listener_;
		ros::Subscriber 		sub_target_;
		ros::Subscriber			sub_odom_;
		ros::Publisher 			pub_cmd_;
		ros::Timer 				cmd_timer_;

		bool first_target_;
		geometry_msgs::PoseStamped	target_;
		TargetPosition 				target_pos_ {TargetPosition::None};
		geometry_msgs::PointStamped user_cmd_;

		const double pi = 3.14159265358979323846;
	
		// Target related parameters
		std::string	ttopic_;
		std::string	frame_id_;
		float 		front_sector_;

		// Command related parameters
		std::string	ctopic_;
		float 		command_angle_left;
		float 		command_angle_right;
		float 		command_angle_forward; 
		float 		command_distance;

		// User related parameters
		float probability_inc_;
		float probability_acc_;
		float min_delivery_time_;
		float max_delivery_time_;
		float refractory_period_;

		std::mt19937 rnd_gen_;
		std::uniform_real_distribution<double> rnd_time_dist_;	
		std::uniform_real_distribution<double> rnd_real_dist_;	

};


#endif
