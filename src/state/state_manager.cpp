#include "rcomponent/state/state_manager.hpp"

namespace rcomponent
{
	StateManager::StateManager(
		rclcpp_lifecycle::LifecycleNode::SharedPtr node,
		std::vector<std::shared_ptr<ManagedPublisherInterface>>& pubs,
		std::vector<std::shared_ptr<ManagedSubscriptorInterface>>& subs) 
	: node_(node),
		logger_(node->get_logger()),
		clock_(node->get_clock())
	{
		
		node_->declare_parameter<bool>("rc_autostart", false);
		node_->declare_parameter<double>("rc_activity_timeout", 10);
		
		autostart_ = node_->get_parameter("rc_autostart").as_bool();
		activity_timeout_ = node_->get_parameter("rc_activity_timeout").as_double();
		
		if (autostart_)
		{
			RCOMPONENT_INFO("Autostart active, configuring and activating node...");
		}	

		state_interfaces_ = std::make_shared<StateInterfaces>(node_);
		lifecycle_manager_ = std::make_shared<LifecycleManager>(node_);
		communication_monitor_ = std::make_shared<CommunicationMonitor>(node_, pubs, subs, activity_timeout_);

		manager_thread_ = std::jthread(
				[this](std::stop_token st){ 
					management_loop(st);
				});

		RCOMPONENT_INFO("Status manager created");
	}

	void StateManager::management_loop(std::stop_token st)
	{
			State lifecycle_state;
			State communication_state;
			uint8_t user_command;
			rclcpp::Time last_time = node_->get_clock()->now();

			while (!st.stop_requested())
			{
				
				// System management

				communication_state = communication_monitor_->get_state();
				lifecycle_state = lifecycle_manager_->get_state();

				if (autostart_)
				{
					RCOMPONENT_WARN("Requesting node start (autostart)");
					lifecycle_manager_->start_node();
					autostart_ = false;
				}

				if (lifecycle_state.id == LifecycleState::PRIMARY_STATE_ACTIVE)
				{
					if (communication_state.id == CommunicationState::COMMUNICATION_STATE_UNHEALTHY)
					{
						RCOMPONENT_WARN("Requesting node pause (communication unhealthy)");
						lifecycle_manager_->pause_node();
					}
				}

				// User requests

				user_command = state_interfaces_->get_command();

				if (user_command == OperationCommand::START)
				{
					RCOMPONENT_WARN("Requesting node start (user request)");
					lifecycle_manager_->start_node();
				}
				else if (user_command == OperationCommand::PAUSE)
				{
					RCOMPONENT_WARN("Requesting node pause (user request)");
					lifecycle_manager_->pause_node();
				}
				else if (user_command == OperationCommand::STOP)
				{
					RCOMPONENT_WARN("Requesting node stop (user request)");
					lifecycle_manager_->stop_node();
				}

				// Publish state

				state_interfaces_->publish(
					lifecycle_state,
					communication_state
				);
				
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
	}

};
		