#include "rcomponent/rcomponent.hpp"

namespace rcomponent
{

Rcomponent::Rcomponent(const std::string& node_name)
	:	rclcpp_lifecycle::LifecycleNode(node_name),
	logger_(rclcpp::get_logger(node_name))
{

	// Declare parameters
	// autostart param is declared in StateManager class
	declare_parameter<double>("rc_loop_frequency", 1.0);

	RCOMPONENT_INFO("Lifecycle RComponent created");
}

void Rcomponent::init()
{
	// Manages the states of an RComponent.
	// Runs in a separate thread to remain active across all ROS 2 lifecycle states.
	rmanager_ = std::make_shared<StateManager>(
		this->shared_from_this(), 
		registered_rc_publishers_, 
		registered_rc_subscriptors_, 
		logger_);
}

CallbackReturn Rcomponent::on_configure(const rclcpp_lifecycle::State &)
{
	RCOMPONENT_INFO("On configure");

	// TODO(robert): Manage return code
	CallbackReturn callback_return = CallbackReturn::SUCCESS;
	
	// Read parameters
	loop_frequency_ = this->get_parameter("rc_loop_frequency").as_double();

	timer_ = this->create_wall_timer(
			std::chrono::milliseconds(static_cast<int>(1000.0 / loop_frequency_)),
			[this]() { rc_loop(); }
	);

	if (callback_return == CallbackReturn::SUCCESS)
	{
		rc_configure();
	}

	return callback_return;
}

CallbackReturn Rcomponent::on_activate(const rclcpp_lifecycle::State &)
{

	RCOMPONENT_INFO("On activate");
	timer_->reset();

	// TODO(robert): Manage return code
	CallbackReturn callback_return = CallbackReturn::SUCCESS;

	for (auto& rc_publisher: registered_rc_publishers_)
	{
		rc_publisher->activate();
	}

	for (auto& rc_subscriptor: registered_rc_subscriptors_)
	{
		rc_subscriptor->activate();
	}

	if (callback_return == CallbackReturn::SUCCESS)
	{
		callback_return = rc_activate();
	}

	return callback_return;
}

CallbackReturn Rcomponent::on_deactivate(const rclcpp_lifecycle::State &)
{
	RCOMPONENT_INFO("On deactivate");
	timer_->cancel();

	// TODO(robert): Manage return code
	CallbackReturn callback_return = CallbackReturn::SUCCESS;

	for (auto& rc_publisher: registered_rc_publishers_)
	{
		rc_publisher->deactivate();
	}

	for (auto& rc_subscriptor: registered_rc_subscriptors_)
	{
		rc_subscriptor->deactivate();
	}

	if (callback_return == CallbackReturn::SUCCESS)
	{
		rc_dectivate();
	}

	return callback_return;
}

CallbackReturn Rcomponent::on_cleanup(const rclcpp_lifecycle::State &)
{
	RCOMPONENT_INFO("On cleanup");
	
	// TODO(robert): Manage return code
	CallbackReturn callback_return = CallbackReturn::SUCCESS;

	for (auto& rc_publisher: registered_rc_publishers_)
	{
		rc_publisher->clear();
	}

	registered_rc_publishers_.clear();

	for (auto& rc_subscriptor: registered_rc_subscriptors_)
	{
		rc_subscriptor->clear();
	}
	registered_rc_subscriptors_.clear();

	if (callback_return == CallbackReturn::SUCCESS)
	{
		callback_return = rc_cleanup();
	}

	return callback_return;
}

CallbackReturn Rcomponent::on_shutdown(const rclcpp_lifecycle::State &)
{
	RCOMPONENT_INFO("On shutdown");

	// TODO(robert): Manage return code
	CallbackReturn callback_return = CallbackReturn::SUCCESS;

	if (callback_return == CallbackReturn::SUCCESS)
	{
		callback_return = rc_shutdown();
	}

	return callback_return;
}

CallbackReturn Rcomponent::on_error(const rclcpp_lifecycle::State &)
{
	RCOMPONENT_INFO("On Error");

	// TODO(robert): Manage return code
	CallbackReturn callback_return = CallbackReturn::SUCCESS;

	if (callback_return == CallbackReturn::SUCCESS)
	{
		callback_return = rc_error();
	}

	return callback_return;
}

}