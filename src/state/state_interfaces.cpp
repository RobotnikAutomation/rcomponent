#include "rcomponent/state/state_interfaces.hpp"

namespace rcomponent
{
	StateInterfaces::StateInterfaces(rclcpp_lifecycle::LifecycleNode::SharedPtr node, rclcpp::Logger logger) 
	: node_(node), 
	logger_(logger)
	{

		state_manager_pub_ = node_->create_publisher<NodeState>(
			node_->get_name() + std::string("/state"), 10);

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

		RCOMPONENT_INFO("Status interfaces created");
	}

	void StateInterfaces::start_callback([[maybe_unused]] TriggerRequest request, TriggerResponse response){

		RCOMPONENT_INFO("rcomponent::state_interfaces: Start request received.");
		operation_command_ = OperationCommand::START;

		response->success = true;
		response->message = " Start request received.";

	}

	void StateInterfaces::stop_callback([[maybe_unused]] TriggerRequest request, TriggerResponse response){

		RCOMPONENT_INFO("rcomponent::state_interfaces: Stop request received.");
		operation_command_ = OperationCommand::STOP;

		response->success = true;
		response->message = " Stop request received.";

	}

	uint8_t StateInterfaces::update()
	{
		
		uint8_t command_to_process = operation_command_;

		// Clear command after consuming
		operation_command_ = OperationCommand::NONE;

		return command_to_process;
	}

	void StateInterfaces::publish(const State& lifecycle_state, 
			const State& communication_state)
	{
		auto msg = NodeState();
		msg.stamp = node_->get_clock()->now();
		
		msg.communication.id = communication_state.id;
		msg.communication.label = communication_state.label;
		msg.lifecycle.id = lifecycle_state.id;
		msg.lifecycle.label = lifecycle_state.label;

		state_manager_pub_->publish(msg);
	}

};
		