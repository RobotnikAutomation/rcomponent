#pragma once

// #include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "rcomponent/types.hpp"
#include "rcomponent/utils/log_macros.hpp"
#include "rcomponent/state/lifecycle/lifecycle_transitions.hpp"

#include "robotnik_common_msgs/msg/node_state.hpp"

namespace rcomponent
{	

	using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
	using LifecycleState = lifecycle_msgs::msg::State;
	using LifecycleTransition = lifecycle_msgs::msg::Transition;

	class LifecycleManager
	{
		public:


			LifecycleManager(rclcpp_lifecycle::LifecycleNode::SharedPtr node, rclcpp::Logger logger);
			
			~LifecycleManager()=default;

			State update(uint8_t operation_command);

		private:

			rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
			rclcpp::Logger logger_;

			bool handle_start(uint8_t current_state_id, uint8_t rcommand);
			bool handle_stop(uint8_t current_state_id, uint8_t rcommand);

			std::shared_ptr<LifecycleTransitions> lifecycle_transitions_;
	};

}