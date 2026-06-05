#include "rcomponent/state/communication/health_check.hpp"

namespace rcomponent
{
	HealthCheck::HealthCheck(
		rclcpp_lifecycle::LifecycleNode::SharedPtr node,
		std::vector<std::shared_ptr<ManagedPublisherInterface>>& pubs,
		std::vector<std::shared_ptr<ManagedSubscriptorInterface>>& subs,
		double timeout)
	: node_(node),
		logger_(node->get_logger()),
		clock_(node->get_clock()),
		pubs_(pubs),
		subs_(subs),
		max_timeout_(timeout),
		offset_timeout_(timeout*0.2)
	{
	}

	bool HealthCheck::subscribers_health()
	{
		bool all_healthy = true;

		for (auto& sub : subs_)
		{
			// Allow a grace period (20% of max_timeout) before enabling inactivity check
			double timeout_sec = sub->time_since_last_activity() - offset_timeout_;
			
			if (timeout_sec > 0 && timeout_sec <= max_timeout_) 
			{
				RCOMPONENT_WARN_THROTTLE(4000, "Topic not received. Timeout (%d/%d) secs: %s",
						static_cast<int>(timeout_sec),
						static_cast<int>(max_timeout_),
						sub->get()->get_topic_name()
				);
			}

			if (timeout_sec > max_timeout_)
			{
				if (all_subs_are_healthy_)  
        {
					RCOMPONENT_WARN("Topic not received. Timeout expired after %d secs: %s",
							static_cast<int>(max_timeout_),
							sub->get()->get_topic_name()
					);
				}
				all_healthy = false;
			}
		}

		all_subs_are_healthy_ = all_healthy;
		
		return all_subs_are_healthy_;
	}

	bool HealthCheck::publishers_health()
	{
		bool all_healthy = true;

		for (auto& pub : pubs_)
		{
			// Allow a grace period (20% of max_timeout) before enabling inactivity check
			double timeout_sec = pub->time_since_last_activity() - offset_timeout_;
			
			if (timeout_sec > 0 && timeout_sec <= max_timeout_) 
			{
				RCOMPONENT_WARN_THROTTLE(2000,"No subscribers detected. Timeout (%d/%d) secs: %s",
						static_cast<int>(timeout_sec),
						static_cast<int>(max_timeout_),
						pub->get()->get_topic_name()
				);
			}
			
			if (timeout_sec > max_timeout_)
			{
				if (all_pubs_are_healthy_)  
        {
					RCOMPONENT_WARN("Subscribers not received. Timeout expired after %d secs: %s",
							static_cast<int>(max_timeout_),
							pub->get()->get_topic_name()
					);
				}
				all_healthy = false;
			}
		}

		all_pubs_are_healthy_ = all_healthy;
		
		return all_pubs_are_healthy_;
	}

};