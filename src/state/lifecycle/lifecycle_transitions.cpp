#include "rcomponent/state/lifecycle/lifecycle_transitions.hpp"

namespace rcomponent
{
	LifecycleTransitions::LifecycleTransitions(rclcpp_lifecycle::LifecycleNode::SharedPtr node) 
	: node_(node),
	logger_(node->get_logger()),
	clock_(node->get_clock())
	{
		// to do
	};

std::string LifecycleTransitions::transition_label(uint8_t id)
	{
		switch(id)
		{
				case LifecycleTransition::TRANSITION_CONFIGURE: return "configure";
				case LifecycleTransition::TRANSITION_ACTIVATE:  return "activate";
				case LifecycleTransition::TRANSITION_CLEANUP: return "cleanup";
				case LifecycleTransition::TRANSITION_DEACTIVATE: return "deactivate";
				default: return "unknown";
		}
	}

	std::string LifecycleTransitions::state_label(uint8_t id)
	{
		switch(id)
		{
				case LifecycleState::PRIMARY_STATE_ACTIVE: return "active";
				case LifecycleState::PRIMARY_STATE_INACTIVE:  return "inactive";
				case LifecycleState::PRIMARY_STATE_UNCONFIGURED: return "unconfigured";
				default: return "unknown";
		}
	}

	std::string LifecycleTransitions::operational_command_label(uint8_t id)
	{
		switch(id)
		{
				case OperationCommand::NONE: return "none";
				case OperationCommand::START:  return "start";
				case OperationCommand::STOP: return "stop";
				default: return "unknown";
		}
	}


	std::string LifecycleTransitions::transition_target(uint8_t id)
	{
		switch(id)
		{
				case LifecycleTransition::TRANSITION_CONFIGURE: return "inactive";
				case LifecycleTransition::TRANSITION_ACTIVATE:  return "active";
				case LifecycleTransition::TRANSITION_CLEANUP: return "unconfigured";
				case LifecycleTransition::TRANSITION_DEACTIVATE: return "inactive";
				default: return "unknown";
		}
	}

	bool LifecycleTransitions::set_transition(uint8_t desired_transition, uint8_t operational_command){

		CallbackReturn cb_ret;

		auto current_state = node_->get_current_state();
		node_->trigger_transition(desired_transition, cb_ret);
		auto new_state = node_->get_current_state();

		if(cb_ret == CallbackReturn::FAILURE || cb_ret == CallbackReturn::ERROR)
		{

			RCOMPONENT_ERROR(
					"rcomponent::lifecycle_manager: Cannot %s node from current state '%s'. Transition '%s' failed. "
					"Callback on_%s() returned %s.",
					operational_command_label(operational_command).c_str(),
					current_state.label().c_str(),
					transition_label(desired_transition).c_str(),
					transition_label(desired_transition).c_str(),
					cb_ret == CallbackReturn::ERROR ? "ERROR" : "FAILURE"
			);

			return false;
		}

		if (new_state.id() == current_state.id())
		{

			std::string available_transitions;
			for (auto & trans: node_->get_available_transitions())
			{
				if (!available_transitions.empty())
				{
					available_transitions += ", ";
				}
				available_transitions += "'" + trans.label() + "'";
			}

			RCOMPONENT_ERROR(
					"rcomponent::lifecycle_manager: Transition '%s' is not available for state '%s'. Available transitions: %s",
					transition_label(desired_transition).c_str(),
					current_state.label().c_str(),
					available_transitions.c_str()
			);

			return false;
		}
		else
		{
			RCOMPONENT_DEBUG(
				"rcomponent::lifecycle_manager: Transition '%s' (%s → %s) succeeded.",
				transition_label(desired_transition).c_str(),
				current_state.label().c_str(),
				transition_target(desired_transition).c_str()
			);
		}

		return true;
	}


	bool LifecycleTransitions::unconfigured_to_active(uint8_t operational_command){

		if (!set_transition(LifecycleTransition::TRANSITION_CONFIGURE, operational_command))
		{
			return false;
		}

		if (!set_transition(LifecycleTransition::TRANSITION_ACTIVATE, operational_command))
		{
			return false;
		}

		return true;
	}


	bool LifecycleTransitions::inactive_to_active(uint8_t operational_command)
	{

		if (!set_transition(LifecycleTransition::TRANSITION_ACTIVATE, operational_command))
		{
			return false;
		}

		return true;
	}

	bool LifecycleTransitions::active_to_unconfigured(uint8_t operational_command)
	{

		if (!set_transition(LifecycleTransition::TRANSITION_DEACTIVATE, operational_command))
		{
			return false;
		}

		if (!set_transition(LifecycleTransition::TRANSITION_CLEANUP, operational_command))
		{
			return false;
		}

		return true;
	}

	bool LifecycleTransitions::inactive_to_unconfigured(uint8_t operational_command)
	{

		if (!set_transition(LifecycleTransition::TRANSITION_CLEANUP, operational_command))
		{
			return false;
		}

		return true;
	}

}