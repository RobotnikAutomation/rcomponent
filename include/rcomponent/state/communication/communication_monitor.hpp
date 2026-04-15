#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "rcomponent/types.hpp"
#include "rcomponent/publisher.hpp"
#include "rcomponent/subscriptor.hpp"
#include "rcomponent/utils/log_macros.hpp"

namespace rcomponent
{	

	class CommunicationMonitor
	{
		public:

			CommunicationMonitor(
				rclcpp_lifecycle::LifecycleNode::SharedPtr node, 
				std::vector<std::shared_ptr<ManagedPublisherInterface>>& pubs,
				std::vector<std::shared_ptr<ManagedSubscriptorInterface>>& subs);
			~CommunicationMonitor()=default;

			State update();

		private:

			rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
			rclcpp::Logger logger_;
			rclcpp::Clock::SharedPtr clock_;

			std::string communication_state_label(uint8_t id);

			uint8_t communication_state_;

			int attempt{0};

			std::vector<std::shared_ptr<ManagedPublisherInterface>>& pubs_;
    	std::vector<std::shared_ptr<ManagedSubscriptorInterface>>& subs_;

	};

}