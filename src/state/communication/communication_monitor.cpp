#include "rcomponent/state/communication/communication_monitor.hpp"

namespace rcomponent
{
	CommunicationMonitor::CommunicationMonitor(
		rclcpp_lifecycle::LifecycleNode::SharedPtr node, 
		std::vector<std::shared_ptr<ManagedPublisherInterface>>& pubs,
		std::vector<std::shared_ptr<ManagedSubscriptorInterface>>& subs,
		rclcpp::Logger logger) 
	: node_(node),
		pubs_(pubs),
		subs_(subs),
		logger_(logger)
	{
		RCOMPONENT_INFO("Communication created");
	}

	std::string CommunicationMonitor::communication_state_label(uint8_t id){

		switch (id)
		{
		case CommunicationState::COMMUNICATION_STATE_UNKNOWN: return "UNKNOWN";
		case CommunicationState::COMMUNICATION_STATE_HEALTHY:  return "HEALTHY";
		case CommunicationState::COMMUNICATION_STATE_UNHEALTHY: return "UNHEALTHY";
		case CommunicationState::COMMUNICATION_STATE_ERROR: return "ERROR";
		default: return "unknown";
		}

	}

	State CommunicationMonitor::update()
	{
		// healthu para pubs puede ser comprobar que hay algo al otro lado esuchando
		// pubs_ y subs_ empaquetar en una clase. Dentro poner el nombre del topic
		// Evaluar si añadir mensaje en el campo del state para indicar el nombre del topic 

		uint8_t communication_state_ = CommunicationState::COMMUNICATION_STATE_HEALTHY;

		for (auto& sub : subs_)
		{
			if (!sub->healthcheck())
			{
				RCLCPP_WARN_THROTTLE(logger_, *(node_->get_clock()), 2000, "Communication unhealthy on topic: %s", sub->topic_name.c_str());
				communication_state_ = CommunicationState::COMMUNICATION_STATE_UNHEALTHY;
			}
		}

		return State(
			communication_state_,
			communication_state_label(communication_state_)
		);
	}


};
		