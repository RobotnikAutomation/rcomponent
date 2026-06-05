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


			LifecycleManager(rclcpp_lifecycle::LifecycleNode::SharedPtr node);
			
			~LifecycleManager()=default;

			bool start_node();
			bool stop_node();
			bool pause_node();
			State get_state();

		private:

			rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
			rclcpp::Logger logger_;
			rclcpp::Clock::SharedPtr clock_;
			
			std::shared_ptr<LifecycleTransitions> lifecycle_transitions_;
	};

}