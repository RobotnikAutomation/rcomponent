#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include "rcomponent/types.hpp"
#include "rcomponent/utils/log_macros.hpp"

#include "robotnik_common_msgs/msg/node_state.hpp"

namespace rcomponent
{	

	using LifecycleState = lifecycle_msgs::msg::State;
	using LifecycleTransition = lifecycle_msgs::msg::Transition;
	using NodeState = robotnik_common_msgs::msg::NodeState;

	class OperationManager
	{
		public:

			OperationManager(rclcpp_lifecycle::LifecycleNode::SharedPtr node, rclcpp::Logger logger);
			~OperationManager()=default;

			State update(uint8_t lifecycle_state);

		private:

			rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
			rclcpp::Logger logger_;

	};

}