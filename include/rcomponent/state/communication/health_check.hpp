	#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "rcomponent/types.hpp"
#include "rcomponent/publisher.hpp"
#include "rcomponent/subscriptor.hpp"
#include "rcomponent/utils/log_macros.hpp"

namespace rcomponent
{	

	class HealthCheck{

		public:
			HealthCheck(
					rclcpp_lifecycle::LifecycleNode::SharedPtr node, 
					std::vector<std::shared_ptr<ManagedPublisherInterface>>& pubs,
					std::vector<std::shared_ptr<ManagedSubscriptorInterface>>& subs,
					double tiemout);
			~HealthCheck()=default;

			bool subscribers_health();
			bool publishers_health();

		private:

			rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
			rclcpp::Logger logger_;
			rclcpp::Clock::SharedPtr clock_;
			std::vector<std::shared_ptr<ManagedPublisherInterface>>& pubs_;
			std::vector<std::shared_ptr<ManagedSubscriptorInterface>>& subs_;
			const double max_timeout_;
			const double offset_timeout_;
			bool all_subs_are_healthy_;
			bool all_pubs_are_healthy_;
		};

}