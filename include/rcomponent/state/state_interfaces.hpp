#pragma once

#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "rcomponent/types.hpp"
#include "rcomponent/state/lifecycle/lifecycle_manager.hpp"
#include "rcomponent/state/operation/operation_manager.hpp"
#include "rcomponent/state/communication/communication_monitor.hpp"
#include "robotnik_common_msgs/msg/node_state.hpp"

namespace rcomponent
{	

	using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
	using TriggerRequest = std::shared_ptr<std_srvs::srv::Trigger::Request>;
	using TriggerResponse = std::shared_ptr<std_srvs::srv::Trigger::Response>;
	using LifecycleState = lifecycle_msgs::msg::State;
	using LifecycleTransition = lifecycle_msgs::msg::Transition;
	using NodeState = robotnik_common_msgs::msg::NodeState;

	class StateInterfaces
	{
		public:

			StateInterfaces(rclcpp_lifecycle::LifecycleNode::SharedPtr node, rclcpp::Logger logger);
			~StateInterfaces()=default;

			uint8_t update();
			void publish(const State& lifecycle_state, const State& operation_state, const State& communication_state);

		private:

			rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
			rclcpp::Logger logger_;

			rclcpp::Publisher<NodeState>::SharedPtr state_manager_pub_;
			rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
			rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
			
			uint8_t operation_command_{OperationCommand::NONE};

			void start_callback([[maybe_unused]] TriggerRequest request, TriggerResponse response);
			void stop_callback([[maybe_unused]] TriggerRequest request, TriggerResponse response);
		
	};

}