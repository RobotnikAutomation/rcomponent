#include "rcomponent/rmanager.hpp"

namespace rcomponent
{
	Rmanager::Rmanager(rclcpp_lifecycle::LifecycleNode::SharedPtr node) 
	: node_(node) 
	{

		manager_thread_ = std::jthread(
				[this](std::stop_token st){ 
					management_loop(st);
				});

		start_service_ = node_->create_service<std_srvs::srv::Trigger>(
			node_->get_name() + std::string("/start"),
			[this](
    		TriggerRequest request, TriggerResponse response)
				{
					start_callback(request, response);
				}
		);

		stop_service_ = node_->create_service<std_srvs::srv::Trigger>(
			node_->get_name() + std::string("/stop"),
			[this](
    		TriggerRequest request, TriggerResponse response)
				{
					stop_callback(request, response);
				}
		);

		rcommand_ = Rcommand::NONE;
	}

	void Rmanager::management_loop(std::stop_token st)
	{		
			rclcpp::Time last_time = node_->get_clock()->now();
			while (!st.stop_requested())
			{
				timer_callback();
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
	}


	void Rmanager::start_callback([[maybe_unused]] TriggerRequest request, TriggerResponse response){

		RCLCPP_INFO(node_->get_logger(),"rcomponent::rmanager: Start request received.");
		rcommand_ = Rcommand::START;

		response->success = true;
		response->message = " Start request received.";

	}



	void Rmanager::stop_callback([[maybe_unused]] TriggerRequest request, TriggerResponse response){

		RCLCPP_INFO(node_->get_logger(), "rcomponent::rmanager: Stop request received.");
		rcommand_ = Rcommand::STOP;

		response->success = true;
		response->message = " Stop request received.";

	}

	std::string Rmanager::transition_label(uint8_t id)
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

	std::string Rmanager::state_label(uint8_t id)
	{
		switch(id)
		{
				case LifecycleState::PRIMARY_STATE_ACTIVE: return "active";
				case LifecycleState::PRIMARY_STATE_INACTIVE:  return "inactive";
				case LifecycleState::PRIMARY_STATE_UNCONFIGURED: return "unconfigured";
				default: return "unknown";
		}
	}

	std::string Rmanager::rcommand_label(uint8_t id)
	{
		switch(id)
		{
				case Rcommand::NONE: return "none";
				case Rcommand::START:  return "start";
				case Rcommand::STOP: return "stop";
				default: return "unknown";
		}
	}


	std::string Rmanager::transition_target(uint8_t id)
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

	bool Rmanager::set_transition(uint8_t desired_transition, uint8_t rcommand){

		CallbackReturn cb_ret;

		auto current_state = node_->get_current_state();
		node_->trigger_transition(desired_transition, cb_ret);
		auto new_state = node_->get_current_state();

		if(cb_ret == CallbackReturn::FAILURE || cb_ret == CallbackReturn::ERROR)
		{

			RCLCPP_ERROR(
					node_->get_logger(),
					"rcomponent::rmanager: Cannot %s node from current state '%s'. Transition '%s' failed. "
					"Callback on_%s() returned %s.",
					rcommand_label(rcommand).c_str(),
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

			RCLCPP_ERROR(
					node_->get_logger(),
					"rcomponent::rmanager: Transition '%s' is not available for state '%s'. Available transitions: %s",
					transition_label(desired_transition).c_str(),
					current_state.label().c_str(),
					available_transitions.c_str()
			);

			return false;
		}
		else
		{
			RCLCPP_DEBUG(
				node_->get_logger(), "rcomponent::rmanager: Transition '%s' (%s → %s) succeeded.",
				transition_label(desired_transition).c_str(),
				current_state.label().c_str(),
				transition_target(desired_transition).c_str()
			);
		}

		return true;
	}


	bool Rmanager::unconfigured_to_active(uint8_t rcommand){

		if (!set_transition(LifecycleTransition::TRANSITION_CONFIGURE, rcommand))
		{
			return false;
		}

		if (!set_transition(LifecycleTransition::TRANSITION_ACTIVATE, rcommand))
		{
			return false;
		}

		return true;
	}


	bool Rmanager::inactive_to_active(uint8_t rcommand)
	{

		if (!set_transition(LifecycleTransition::TRANSITION_ACTIVATE, rcommand))
		{
			return false;
		}

		return true;
	}

	bool Rmanager::active_to_unconfigured(uint8_t rcommand)
	{

		if (!set_transition(LifecycleTransition::TRANSITION_DEACTIVATE, rcommand))
		{
			return false;
		}

		if (!set_transition(LifecycleTransition::TRANSITION_CLEANUP, rcommand))
		{
			return false;
		}

		return true;
	}

	bool Rmanager::inactive_to_unconfigured(uint8_t rcommand)
	{

		if (!set_transition(LifecycleTransition::TRANSITION_CLEANUP, rcommand))
		{
			return false;
		}

		return true;
	}

	bool Rmanager::handle_start(uint8_t current_state, uint8_t rcommand){

		bool success;

		switch (current_state)
		{
		case LifecycleState::PRIMARY_STATE_UNCONFIGURED:
			success = unconfigured_to_active(rcommand);
			break;
		
		case LifecycleState::PRIMARY_STATE_INACTIVE:
			success = inactive_to_active(rcommand);
			break;

		default:
			RCLCPP_ERROR(
				node_->get_logger(), "rcomponent::rmanager: Node must be in '%s' or '%s' state before starting.",
				state_label(LifecycleState::PRIMARY_STATE_UNCONFIGURED).c_str(),
				state_label(LifecycleState::PRIMARY_STATE_INACTIVE).c_str()
			);
			success = false;
			break;
		}

		return success;
	}


	bool Rmanager::handle_stop(uint8_t current_state, uint8_t rcommand){

		bool success;

		switch (current_state)
		{
		case LifecycleState::PRIMARY_STATE_ACTIVE:
			success = active_to_unconfigured(rcommand);
			break;
		
		case LifecycleState::PRIMARY_STATE_INACTIVE:
			success = inactive_to_unconfigured(rcommand);
			break;

		default:
			RCLCPP_ERROR(
					node_->get_logger(),
					"rcomponent::rmanager: Cannot stop node from current state '%s'. Expected: '%s' or '%s'.",
					state_label(current_state).c_str(),
					state_label(LifecycleState::PRIMARY_STATE_ACTIVE).c_str(),
					state_label(LifecycleState::PRIMARY_STATE_INACTIVE).c_str()
			);
			success = false;
			break;
		}

		return success;
	}

	void Rmanager::timer_callback()
	{

		if (rcommand_ != Rcommand::NONE)
		{

			uint8_t current_state = node_->get_current_state().id();

			RCLCPP_INFO(node_->get_logger(), "rcomponent::rmanager: Handling %s command.", 
				rcommand_label(rcommand_).c_str());

			switch (rcommand_)
			{
				case START:

					if (handle_start(current_state, rcommand_))
					{
        		RCLCPP_INFO(node_->get_logger(), "rcomponent::rmanager: Node successfully started!");
					}
					else
					{
						RCLCPP_ERROR(node_->get_logger(), "rcomponent::rmanager: Failed to start node.");
					}
					break;

				case STOP:

					if (handle_stop(current_state, rcommand_))
					{
        		RCLCPP_INFO(node_->get_logger(), "rcomponent::rmanager: Node successfully stopped!");
					}
					else
					{
						RCLCPP_ERROR(node_->get_logger(), "rcomponent::rmanager: Failed to stop node.");
					}
					break;

				default:

					RCLCPP_ERROR(node_->get_logger(), "rcomponent::rmanager: Unknown rcomponent command: '%d'", rcommand_);
					break;
			}

			rcommand_ = NONE;
		}
	}
}
