#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "rcomponent/types.hpp"
#include "rcomponent/utils/log_macros.hpp"

namespace rcomponent
{	

	class CommunicationMonitor
	{
		public:

			CommunicationMonitor(rclcpp_lifecycle::LifecycleNode::SharedPtr node, rclcpp::Logger logger);
			~CommunicationMonitor()=default;

			State update();

		private:

			rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
			rclcpp::Logger logger_;

	};

}