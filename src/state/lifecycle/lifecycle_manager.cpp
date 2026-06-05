#include "rcomponent/state/lifecycle/lifecycle_manager.hpp"

namespace rcomponent
{
	LifecycleManager::LifecycleManager(rclcpp_lifecycle::LifecycleNode::SharedPtr node) 
	: node_(node),
	logger_(node->get_logger()),
	clock_(node->get_clock())
	{
		lifecycle_transitions_ = std::make_shared<LifecycleTransitions>(node_);

		RCOMPONENT_INFO("Lifecycle created");
	}

	bool LifecycleManager::start_node()
	{
		bool success;

		RCOMPONENT_INFO("rcomponent::lifecycle_manager: Starting node...");

		switch (node_->get_current_state().id())
		{
		case LifecycleState::PRIMARY_STATE_UNCONFIGURED:
			success = lifecycle_transitions_->unconfigured_to_active();
			break;
		
		case LifecycleState::PRIMARY_STATE_INACTIVE:
			success = lifecycle_transitions_->inactive_to_active();
			break;

		case LifecycleState::PRIMARY_STATE_ACTIVE:
			RCOMPONENT_WARN("rcomponent::lifecycle_manager: Node is already started");
			success = true;
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

		if (success)
		{
			RCOMPONENT_INFO("rcomponent::lifecycle_manager: Node successfully started!");
		}
		else
		{
			RCOMPONENT_ERROR("rcomponent::lifecycle_manager: Failed to start node.");
		}

		return success;
	}


	bool LifecycleManager::stop_node()
	{
		bool success;

		RCOMPONENT_INFO("rcomponent::lifecycle_manager: Stopping node...");

		switch (node_->get_current_state().id())
		{
		case LifecycleState::PRIMARY_STATE_ACTIVE:
			success = lifecycle_transitions_->active_to_unconfigured();
			break;
		
		case LifecycleState::PRIMARY_STATE_INACTIVE:
			success = lifecycle_transitions_->inactive_to_unconfigured();
			break;

		case LifecycleState::PRIMARY_STATE_UNCONFIGURED:
			RCOMPONENT_WARN("rcomponent::lifecycle_manager: Node is already stopped");
			success = true;
			break;

		default:
			RCOMPONENT_ERROR(
					"rcomponent::lifecycle_manager: Cannot stop node from current state '%s'. Expected: '%s' or '%s'.",
					lifecycle_transitions_->state_label(node_->get_current_state().id()).c_str(),
					lifecycle_transitions_->state_label(LifecycleState::PRIMARY_STATE_ACTIVE).c_str(),
					lifecycle_transitions_->state_label(LifecycleState::PRIMARY_STATE_INACTIVE).c_str()
			);
			success = false;
			break;
		}

		if (success)
		{
			RCOMPONENT_INFO("rcomponent::lifecycle_manager: Node successfully stopped!");
		}
		else
		{
			RCOMPONENT_ERROR("rcomponent::lifecycle_manager: Failed to stop node.");
		}

		return success;
	}

	bool LifecycleManager::pause_node()
	{
		bool success;

		RCOMPONENT_INFO("rcomponent::lifecycle_manager: Pausing node...");

		switch (node_->get_current_state().id())
		{
		case LifecycleState::PRIMARY_STATE_UNCONFIGURED:
			success = lifecycle_transitions_->unconfigured_to_inactive();
			break;
		
		case LifecycleState::PRIMARY_STATE_ACTIVE:
			success = lifecycle_transitions_->active_to_inactive();
			break;

		case LifecycleState::PRIMARY_STATE_INACTIVE:
			RCOMPONENT_WARN("rcomponent::lifecycle_manager: Node is already paused");
			success = true;
			break;

		default:
			RCOMPONENT_ERROR(
					"rcomponent::lifecycle_manager: Cannot pause node from current state '%s'. Expected: '%s' or '%s'.",
					lifecycle_transitions_->state_label(node_->get_current_state().id()).c_str(),
					lifecycle_transitions_->state_label(LifecycleState::PRIMARY_STATE_ACTIVE).c_str(),
					lifecycle_transitions_->state_label(LifecycleState::PRIMARY_STATE_UNCONFIGURED).c_str()
			);
			success = false;
			break;
		}

		if (success)
		{
			RCOMPONENT_INFO("rcomponent::lifecycle_manager: Node successfully paused!");
		}
		else
		{
			RCOMPONENT_ERROR("rcomponent::lifecycle_manager: Failed to pause node.");
		}

		return success;
	}

	State LifecycleManager::get_state()
	{
		return State(
				node_->get_current_state().id(),
				node_->get_current_state().label()
			);
	}

}
