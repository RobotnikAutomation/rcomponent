#pragma once

#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "rcomponent/types.hpp"
#include "rcomponent/utils/log_macros.hpp"

#include "robotnik_common_msgs/msg/node_state.hpp"

namespace rcomponent
{	

	using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
	using LifecycleState = lifecycle_msgs::msg::State;
	using LifecycleTransition = lifecycle_msgs::msg::Transition;


	class LifecycleTransitions
	{
		public:

			LifecycleTransitions(rclcpp_lifecycle::LifecycleNode::SharedPtr node);
			~LifecycleTransitions()=default;

			std::string transition_label(uint8_t id);
			std::string state_label(uint8_t id);
			std::string operational_command_label(uint8_t id);
			std::string transition_target(uint8_t id);

			bool set_transition(uint8_t desired_transition, uint8_t operational_command);

			bool unconfigured_to_active(uint8_t operational_command);
			bool inactive_to_active(uint8_t operational_command);
			bool active_to_unconfigured(uint8_t operational_command);
			bool inactive_to_unconfigured(uint8_t operational_command);

		private:

			rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
			rclcpp::Logger logger_;
			rclcpp::Clock::SharedPtr clock_;

	};

}