#include "rcomponent/state/communication/communication_monitor.hpp"

namespace rcomponent
{
	CommunicationMonitor::CommunicationMonitor(
		rclcpp_lifecycle::LifecycleNode::SharedPtr node, 
		std::vector<std::shared_ptr<ManagedPublisherInterface>>& pubs,
		std::vector<std::shared_ptr<ManagedSubscriptorInterface>>& subs,
		double timeout)
	: node_(node),
		logger_(node->get_logger()),
		clock_(node->get_clock()),
		pubs_(pubs),
		subs_(subs)
	{
		RCOMPONENT_INFO("Communication created");

		healthcheck_ = std::make_shared<HealthCheck>(node_, pubs, subs, timeout);

		communication_state_ = CommunicationState::COMMUNICATION_STATE_UNKNOWN;

		last_health_check_time_ = node_->now();
	}

	std::string CommunicationMonitor::communication_state_label(uint8_t id){

		switch (id)
		{
		case CommunicationState::COMMUNICATION_STATE_UNKNOWN: return "unknown";
		case CommunicationState::COMMUNICATION_STATE_HEALTHY:  return "healthy";
		case CommunicationState::COMMUNICATION_STATE_UNHEALTHY: return "unhealthy";
		case CommunicationState::COMMUNICATION_STATE_ERROR: return "error";
		default: return "unknown";
		}

	}

	State CommunicationMonitor::get_state()
	{
		if (node_->get_current_state().id() == LifecycleState::PRIMARY_STATE_UNCONFIGURED)
		{
			communication_state_ = CommunicationState::COMMUNICATION_STATE_UNKNOWN;
		}
		else if (node_->get_current_state().id() == LifecycleState::PRIMARY_STATE_ACTIVE)
		{
			if (!healthcheck_->subscribers_health() || !healthcheck_->publishers_health())
			{
				communication_state_ = CommunicationState::COMMUNICATION_STATE_UNHEALTHY;
				RCOMPONENT_WARN_THROTTLE(4000,"Communication is state unhealthy");
			}
			else
			{
				communication_state_ = CommunicationState::COMMUNICATION_STATE_HEALTHY;
			}
		}
		
		return State(
			communication_state_,
			communication_state_label(communication_state_)
		);
	}
};