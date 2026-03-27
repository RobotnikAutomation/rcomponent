#pragma once

#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "rcomponent/types.hpp"
#include "rcomponent/publisher.hpp"
#include "rcomponent/subscriptor.hpp"
#include "rcomponent/state/lifecycle/lifecycle_manager.hpp"
#include "rcomponent/state/communication/communication_monitor.hpp"
#include "rcomponent/state/state_interfaces.hpp"
#include "robotnik_common_msgs/msg/node_state.hpp"

namespace rcomponent
{	

	using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
	using TriggerRequest = std::shared_ptr<std_srvs::srv::Trigger::Request>;
	using TriggerResponse = std::shared_ptr<std_srvs::srv::Trigger::Response>;
	using LifecycleState = lifecycle_msgs::msg::State;
	using LifecycleTransition = lifecycle_msgs::msg::Transition;
	using NodeState = robotnik_common_msgs::msg::NodeState;


	class StateManager
	{
		public:

			StateManager(
				rclcpp_lifecycle::LifecycleNode::SharedPtr node,
				std::vector<std::shared_ptr<ManagedPublisherInterface>>& pubs,
				std::vector<std::shared_ptr<ManagedSubscriptorInterface>>& subs,
				rclcpp::Logger logger);
			~StateManager()=default;

		private:

    	std::jthread manager_thread_;

			rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
			rclcpp::Logger logger_;
			rclcpp::Publisher<NodeState>::SharedPtr state_manager_pub_;
			
			rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
			rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
			
			std::shared_ptr<LifecycleManager> lifecycle_manager_;
			std::shared_ptr<CommunicationMonitor> communication_monitor_;
			std::shared_ptr<StateInterfaces> state_interfaces_;

			bool autostart_{false};

			void management_loop(std::stop_token st);

	};

}