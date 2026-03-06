#include "rcomponent/state/lifecycle/lifecycle_manager.hpp"

namespace rcomponent
{
	LifecycleManager::LifecycleManager(rclcpp_lifecycle::LifecycleNode::SharedPtr node, rclcpp::Logger logger) 
	: node_(node),
	logger_(logger)
	{
		
		lifecycle_transitions_ = std::make_shared<LifecycleTransitions>(node_, logger_);

		RCOMPONENT_INFO("Lifecycle created");

	}

	bool LifecycleManager::handle_start(uint8_t current_state_id, uint8_t rcommand){

		bool success;

		switch (current_state_id)
		{
		case LifecycleState::PRIMARY_STATE_UNCONFIGURED:
			success = lifecycle_transitions_->unconfigured_to_active(rcommand);
			break;
		
		case LifecycleState::PRIMARY_STATE_INACTIVE:
			success = lifecycle_transitions_->inactive_to_active(rcommand);
			break;

		default:
			RCOMPONENT_ERROR(
				"rcomponent::lifecycle_manager: Node must be in '%s' or '%s' state before starting.",
				lifecycle_transitions_->state_label(LifecycleState::PRIMARY_STATE_UNCONFIGURED).c_str(),
				lifecycle_transitions_->state_label(LifecycleState::PRIMARY_STATE_INACTIVE).c_str()
			);
			success = false;
			break;
		}

		return success;
	}


	bool LifecycleManager::handle_stop(uint8_t current_state_id, uint8_t rcommand){

		bool success;

		switch (current_state_id)
		{
		case LifecycleState::PRIMARY_STATE_ACTIVE:
			success = lifecycle_transitions_->active_to_unconfigured(rcommand);
			break;
		
		case LifecycleState::PRIMARY_STATE_INACTIVE:
			success = lifecycle_transitions_->inactive_to_unconfigured(rcommand);
			break;

		default:
			RCOMPONENT_ERROR(
					"rcomponent::lifecycle_manager: Cannot stop node from current state '%s'. Expected: '%s' or '%s'.",
					lifecycle_transitions_->state_label(current_state_id).c_str(),
					lifecycle_transitions_->state_label(LifecycleState::PRIMARY_STATE_ACTIVE).c_str(),
					lifecycle_transitions_->state_label(LifecycleState::PRIMARY_STATE_INACTIVE).c_str()
			);
			success = false;
			break;
		}

		return success;
	}

	State LifecycleManager::update(uint8_t operation_command)
	{
		
		//rclcpp_lifecycle::State current_state = node_->get_current_state();

		State current_state(
			node_->get_current_state().id(),
			node_->get_current_state().label()
		);

		if (operation_command != OperationCommand::NONE)
		{

			RCOMPONENT_INFO("rcomponent::lifecycle_manager: Handling %s command.", 
				lifecycle_transitions_->operational_command_label(operation_command).c_str());

			switch (operation_command)
			{
				case START:

					if (handle_start(current_state.id, operation_command))
					{
        		RCOMPONENT_INFO("rcomponent::lifecycle_manager: Node successfully started!");
					}
					else
					{
						RCOMPONENT_ERROR("rcomponent::lifecycle_manager: Failed to start node.");
					}
					break;

				case STOP:

					if (handle_stop(current_state.id, operation_command))
					{
        		RCOMPONENT_INFO("rcomponent::lifecycle_manager: Node successfully stopped!");
					}
					else
					{
						RCOMPONENT_ERROR("rcomponent::lifecycle_manager: Failed to stop node.");
					}
					break;

				default:

					RCOMPONENT_ERROR("rcomponent::lifecycle_manager: Unknown rcomponent command: '%d'", operation_command);
					break;
			}

		}

		return current_state;
	}


}
