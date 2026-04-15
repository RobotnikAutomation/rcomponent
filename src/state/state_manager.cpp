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
		autostart_ = node_->get_parameter("rc_autostart").as_bool();

		if (autostart_)
		{
			RCOMPONENT_INFO("Autostart active, configuring and activating node...");
		}	

		state_interfaces_ = std::make_shared<StateInterfaces>(node_);
		lifecycle_manager_ = std::make_shared<LifecycleManager>(node_);
		communication_monitor_ = std::make_shared<CommunicationMonitor>(node_, pubs, subs);

		manager_thread_ = std::jthread(
				[this](std::stop_token st){ 
					management_loop(st);
				});

		RCOMPONENT_INFO("Status manager created");
	}

	void StateManager::management_loop(std::stop_token st)
	{		
			State lifecycle_state;
			State communication_monitor;
			uint8_t operation_command;
			rclcpp::Time last_time = node_->get_clock()->now();

			while (!st.stop_requested())
			{

				if (autostart_)
				{
					operation_command = OperationCommand::START;
					autostart_ = false;
				}
				else
				{
					operation_command = state_interfaces_->update();
				}

				communication_monitor = communication_monitor_->update();

				lifecycle_state = lifecycle_manager_->update(operation_command);

				if (communication_monitor.id == CommunicationState::COMMUNICATION_STATE_UNHEALTHY)
				{
					if (lifecycle_state.id == LifecycleState::PRIMARY_STATE_ACTIVE)
					{
						// Mandar a deactivate. Evaliar si implementar PAUSE. Desde ahi se puede
						// hacer que el nodo se autorecupere porque el callback sigue funcionando en ese estado
						operation_command = OperationCommand::STOP;
						lifecycle_state = lifecycle_manager_->update(operation_command);
					}
				}

				state_interfaces_->publish(
					lifecycle_state,
					communication_monitor
				);

				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
	}

};
		