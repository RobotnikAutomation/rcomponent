#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "rcomponent/types.hpp"
#include "rcomponent/publisher.hpp"
#include "rcomponent/subscriptor.hpp"
#include "rcomponent/utils/log_macros.hpp"
#include "rcomponent/state/communication/health_check.hpp"

namespace rcomponent
{	
	using LifecycleState = lifecycle_msgs::msg::State;

	class CommunicationMonitor
	{
		public:

			CommunicationMonitor(
				rclcpp_lifecycle::LifecycleNode::SharedPtr node, 
				std::vector<std::shared_ptr<ManagedPublisherInterface>>& pubs,
				std::vector<std::shared_ptr<ManagedSubscriptorInterface>>& subs,
				double timeout);
			~CommunicationMonitor()=default;

			State get_state();

		private:

			rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
			rclcpp::Logger logger_;
			rclcpp::Clock::SharedPtr clock_;

			std::shared_ptr<HealthCheck> healthcheck_;

			std::string communication_state_label(uint8_t id);

			uint8_t communication_state_;

			rclcpp::Time last_health_check_time_;
			
			std::vector<std::shared_ptr<ManagedPublisherInterface>>& pubs_;
    	std::vector<std::shared_ptr<ManagedSubscriptorInterface>>& subs_;

	};

}