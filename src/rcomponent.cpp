#include "rcomponent/rcomponent.hpp"

namespace rcomponent
{

Rcomponent::Rcomponent(const std::string& node_name)
	:	rclcpp_lifecycle::LifecycleNode(node_name),
	logger_(this->get_logger()),
	clock_(this->get_clock())
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
		registered_rc_subscriptors_);
}

CallbackReturn Rcomponent::on_configure(const rclcpp_lifecycle::State &)
{
	RCOMPONENT_INFO("On configure");

	auto ret = rc_configure();
	if (ret != CallbackReturn::SUCCESS)
	{
		RCLCPP_ERROR(get_logger(), "rc_configure() operation failed");
		return ret;
	}

	// Read parameters
	if (!this->get_parameter("rc_loop_frequency", loop_frequency_))
	{
		RCLCPP_ERROR(get_logger(), "Parameter 'rc_loop_frequency' not set");
		return CallbackReturn::FAILURE;
	}

	if (loop_frequency_ <= 0.0)
	{
		RCLCPP_ERROR(get_logger(), "Invalid loop frequency: %.2f", loop_frequency_);
		return CallbackReturn::FAILURE;
	}

	timer_ = this->create_wall_timer(
			std::chrono::milliseconds(static_cast<int>(1000.0 / loop_frequency_)),
			[this]() { rc_loop(); }
	);

	return  CallbackReturn::SUCCESS;
}

CallbackReturn Rcomponent::on_activate(const rclcpp_lifecycle::State &)
{

	RCOMPONENT_INFO("On activate");

	auto ret = rc_activate();
	if (ret != CallbackReturn::SUCCESS)
	{
		RCLCPP_ERROR(get_logger(), "rc_activate() operation failed");
		return ret;
	}

	if (!timer_)
	{
		RCLCPP_ERROR(get_logger(), "Timer not initialized");
		return CallbackReturn::FAILURE;
	}

	for (auto& rc_publisher: registered_rc_publishers_)
	{
		rc_publisher->activate();
	}

	for (auto& rc_subscriptor: registered_rc_subscriptors_)
	{
		rc_subscriptor->activate();
	}

	timer_->reset();

	return CallbackReturn::SUCCESS;
}

CallbackReturn Rcomponent::on_deactivate(const rclcpp_lifecycle::State &)
{
	RCOMPONENT_INFO("On deactivate");
	
	auto ret = rc_deactivate();
	if (ret != CallbackReturn::SUCCESS)
	{
		RCLCPP_ERROR(get_logger(), "rc_deactivate() operation failed");
		return ret;
	}

	if (timer_)
	{
		timer_->cancel();
	}
	else
	{
		RCLCPP_WARN(get_logger(), "Timer was not initialized (nullptr), skipping cancel");
	}

	for (auto& rc_publisher: registered_rc_publishers_)
	{
		rc_publisher->deactivate();
	}

	for (auto& rc_subscriptor: registered_rc_subscriptors_)
	{
		rc_subscriptor->deactivate();
	}

	return CallbackReturn::SUCCESS;
}

CallbackReturn Rcomponent::on_cleanup(const rclcpp_lifecycle::State &)
{
	RCOMPONENT_INFO("On cleanup");

	auto ret = rc_cleanup();
	if (ret != CallbackReturn::SUCCESS)
	{
		RCLCPP_ERROR(get_logger(), "rc_cleanup() operation failed");
		return ret;
	}
	
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

	return CallbackReturn::SUCCESS;
}

CallbackReturn Rcomponent::on_shutdown(const rclcpp_lifecycle::State &)
{
	RCOMPONENT_INFO("On shutdown");

	auto ret = rc_shutdown();
	if (ret != CallbackReturn::SUCCESS)
	{
		RCLCPP_ERROR(get_logger(), "rc_shutdown() operation failed");
		return ret;
	}

	return CallbackReturn::SUCCESS;
}

CallbackReturn Rcomponent::on_error(const rclcpp_lifecycle::State &)
{
	RCOMPONENT_INFO("On Error");

	auto ret = rc_error();
	if (ret != CallbackReturn::SUCCESS)
	{
		RCLCPP_ERROR(get_logger(), "rc_error() operation failed");
		return ret;
	}
	
	return CallbackReturn::SUCCESS;
}

}