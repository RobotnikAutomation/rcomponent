#include "rcomponent/state/communication/communication_monitor.hpp"

namespace rcomponent
{
	CommunicationMonitor::CommunicationMonitor(rclcpp_lifecycle::LifecycleNode::SharedPtr node, rclcpp::Logger logger) 
	: node_(node),
	logger_(logger)
	{
		RCOMPONENT_INFO("Communication created");
	}

	State CommunicationMonitor::update()
	{
		
		return State(
			CommunicationState::COMMUNICATION_STATE_HEALTHY,
			"HEALTHY"
		);
	}


};
		