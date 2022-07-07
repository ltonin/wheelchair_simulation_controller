#ifndef WHEELCHAIR_COMMAND_GENERATOR_HPP
#define WHEELCHAIR_COMMAND_GENERATOR_HPP

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>

enum class UserCommand {NONE, LEFT, FORWARD, RIGHT};

class CommandGenerator {

	public:
		CommandGenerator(void);
		~CommandGenerator(void);
		bool configure(void);
		bool run(void);

	private:
		void on_received_target(const geometry_msgs::PoseStamped& msg);

		void set_correct_command(void);

	private:
		ros::NodeHandle			nh_;
		ros::NodeHandle			p_nh_;
		tf::TransformListener	listener_;
		ros::Subscriber 		sub_target_;
		ros::Subscriber			sub_odom_;
		std::string				ttopic_;
		std::string				frame_id_;

		bool first_target_;
		geometry_msgs::PoseStamped	target_;
		UserCommand 				correct_cmd_ {UserCommand::NONE};

};


#endif
