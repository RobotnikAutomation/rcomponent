#include "rcomponent/state/operation/operation_manager.hpp"

namespace rcomponent
{
	OperationManager::OperationManager(rclcpp_lifecycle::LifecycleNode::SharedPtr node, rclcpp::Logger logger) 
	: node_(node),
	logger_(logger)
	{
	RCOMPONENT_INFO("Operation created");
	}

	State OperationManager::update(uint8_t lifecycle_state_id)
	{		
		State operation_state;

		if (lifecycle_state_id == LifecycleState::PRIMARY_STATE_UNCONFIGURED)
		{
			operation_state.id = NodeState::OPERATION_STATE_INIT;
			operation_state.label = "INIT";
		}
		else if (lifecycle_state_id == LifecycleState::PRIMARY_STATE_INACTIVE)
		{
			operation_state.id = NodeState::OPERATION_STATE_STANDBY;
			operation_state.label = "STANDBY";
		}
		else if (lifecycle_state_id == LifecycleState::PRIMARY_STATE_ACTIVE)
		{
			operation_state.id = NodeState::OPERATION_STATE_READY; // Podria llamarse RUNNING
			operation_state.label = "READY";
		}
		else if (lifecycle_state_id == LifecycleState::PRIMARY_STATE_FINALIZED)
		{
			operation_state.id = NodeState::OPERATION_STATE_SHUTDOWN;
			operation_state.label = "SHUTDOWN";
		}
		else
		{
			operation_state.id = NodeState::OPERATION_STATE_UNKNOWN;
			operation_state.label = "UNKNOWN";
		}
		return operation_state;
	}

};
		