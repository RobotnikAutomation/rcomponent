#include "rcomponent/state/state_manager.hpp"

namespace rcomponent
{
	StateManager::StateManager(rclcpp_lifecycle::LifecycleNode::SharedPtr node, rclcpp::Logger logger) 
	: node_(node), 
	logger_(logger)
	{
		
		state_interfaces_ = std::make_shared<StateInterfaces>(node_, logger_);
		lifecycle_manager_ = std::make_shared<LifecycleManager>(node_, logger_);
		operation_manager_ = std::make_shared<OperationManager>(node_, logger_);
		communication_monitor_ = std::make_shared<CommunicationMonitor>(node_, logger_);

		manager_thread_ = std::jthread(
				[this](std::stop_token st){ 
					management_loop(st);
				});

		RCOMPONENT_INFO("Status manager created");
	}

	void StateManager::management_loop(std::stop_token st)
	{		
			State lifecycle_state;
			State operation_manager;
			State communication_monitor;
			uint8_t operation_command;
			rclcpp::Time last_time = node_->get_clock()->now();
			
			while (!st.stop_requested())
			{
				operation_command = state_interfaces_->update();
				lifecycle_state = lifecycle_manager_->update(operation_command);
				operation_manager = operation_manager_->update(lifecycle_state.id);
				communication_monitor = communication_monitor_->update();

				operation_command_ = OperationCommand::NONE;

				state_interfaces_->publish(
					lifecycle_state,
					operation_manager,
					communication_monitor
				);

				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
	}

};
		