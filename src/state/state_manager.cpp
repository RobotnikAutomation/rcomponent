#include "rcomponent/state/state_manager.hpp"

namespace rcomponent
{
	StateManager::StateManager(rclcpp_lifecycle::LifecycleNode::SharedPtr node, rclcpp::Logger logger) 
	: node_(node), 
	logger_(logger)
	{
		
		state_interfaces_ = std::make_shared<StateInterfaces>(node_, logger_);
		lifecycle_manager_ = std::make_shared<LifecycleManager>(node_, logger_);
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
			State communication_monitor;
			uint8_t operation_command;
			rclcpp::Time last_time = node_->get_clock()->now();
			
			while (!st.stop_requested())
			{
				operation_command = state_interfaces_->update();
				lifecycle_state = lifecycle_manager_->update(operation_command);
				communication_monitor = communication_monitor_->update();

				state_interfaces_->publish(
					lifecycle_state,
					communication_monitor
				);

				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
	}

};
		