#include "rcomponent/state/communication/communication_monitor.hpp"

namespace rcomponent
{
	CommunicationMonitor::CommunicationMonitor(
		rclcpp_lifecycle::LifecycleNode::SharedPtr node, 
		std::vector<std::shared_ptr<ManagedPublisherInterface>>& pubs,
		std::vector<std::shared_ptr<ManagedSubscriptorInterface>>& subs) 
	: node_(node),
		logger_(node->get_logger()),
		clock_(node->get_clock()),
		pubs_(pubs),
		subs_(subs)
	{
		RCOMPONENT_INFO("Communication created");

		communication_state_ = CommunicationState::COMMUNICATION_STATE_HEALTHY;
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

	State CommunicationMonitor::update()
	{
		// Añadir heatlcheck el publisher que compruebe que hay algo al otro lado esuchando
		// Añadir parametro required que haga o no parar el nodo
		// attemps, healtcheck timeout, y required deben ser parametros configurables
		// Añadir mensaje en el campo del state para indicar el nombre del topic 
		// Mover de subscriptor/publisher para usarse en communicator monitor para dentro de la clase healtcheck
		// Revisar estructura para integracion clara entre monitor y lifecycle manager.

		for (auto& sub : subs_)
		{
			if (!sub->healthcheck())
			{
				attempt++;
				RCOMPONENT_WARN_THROTTLE(2000, "Topic not received. Attempt (%d/100): %s", attempt, sub->get()->get_topic_name());
				if (attempt >= 100)
				{	
					communication_state_ = CommunicationState::COMMUNICATION_STATE_UNHEALTHY;
					RCOMPONENT_WARN("Communication state unhealthy: No messages received in topic '%s' for 100 attempts.", sub->get()->get_topic_name());
				}
			}
			else
			{
				communication_state_ = CommunicationState::COMMUNICATION_STATE_HEALTHY;
				attempt = 0;
			}
		}

		return State(
			communication_state_,
			communication_state_label(communication_state_)
		);
	}


};
		