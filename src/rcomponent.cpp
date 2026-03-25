#include "rcomponent/rcomponent.hpp"

namespace rcomponent
{

Rcomponent::Rcomponent(const std::string& node_name)
	:	rclcpp_lifecycle::LifecycleNode(node_name),
	logger_(rclcpp::get_logger(node_name))
{

	// Declare parameters
	// autostart param is declared in StateManager class
	declare_parameter<double>("frequency", 1.0);

	RCOMPONENT_INFO("Lifecycle RComponent created");
}

void Rcomponent::init()
{
	
	// Manages the states of an RComponent.
	// Runs in a separate thread to remain active across all ROS 2 lifecycle states.
	rmanager_ = std::make_shared<StateManager>(this->shared_from_this(), logger_);

}

void Rcomponent::control_loop()
{

}

CallbackReturn Rcomponent::on_configure(const rclcpp_lifecycle::State &)
{

	// Read parameters
	frequency_ = this->get_parameter("frequency").as_double();

	timer_ = this->create_wall_timer(
			std::chrono::milliseconds(static_cast<int>(1000.0 / frequency_)),
			[this]() { control_loop(); }
	);

	RCOMPONENT_INFO("On configure");
	return CallbackReturn::SUCCESS;
}

CallbackReturn Rcomponent::on_activate(const rclcpp_lifecycle::State &)
{
	timer_->reset();

	// Run node
	RCOMPONENT_INFO("On activate");
	return CallbackReturn::SUCCESS;
}

CallbackReturn Rcomponent::on_deactivate(const rclcpp_lifecycle::State &)
{
	timer_->cancel();

	// Pause node
	RCOMPONENT_INFO("On deactivate");
	return CallbackReturn::SUCCESS;
}

CallbackReturn Rcomponent::on_cleanup(const rclcpp_lifecycle::State &)
{
	// Stop node
	RCOMPONENT_INFO("On cleanup");
	return CallbackReturn::SUCCESS;
}

CallbackReturn Rcomponent::on_shutdown(const rclcpp_lifecycle::State &)
{
	// Shutdown node
	RCOMPONENT_INFO("On shutdown");
	return CallbackReturn::SUCCESS;
}

CallbackReturn Rcomponent::on_error(const rclcpp_lifecycle::State &)
{
	// Error node
	RCOMPONENT_INFO("On Error");
	return CallbackReturn::SUCCESS;
}

}