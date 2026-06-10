#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "rcomponent/publisher.hpp"
#include "rcomponent/subscriptor.hpp"
#include "rcomponent/utils/factory.hpp"
#include "rcomponent/utils/log_macros.hpp"
#include "rcomponent/state/state_manager.hpp"

// Comprobar clock si use_sim es true
// Añadir readme 
// Revisar comentarios headers y dependencias
// Añadir test minimo
// Añadir creador basico de pubs y subs
// Añadir creador de single/multithread

namespace rcomponent
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Rcomponent : public rclcpp_lifecycle::LifecycleNode
{

	public:

		Rcomponent() = delete;
		explicit Rcomponent(const std::string& node_name);
	
		void init();

		rclcpp::Logger logger_;
		rclcpp::Clock::SharedPtr clock_;

	protected:

		template<typename T>
		T get_rc_param(const std::string & name, const T & default_value)
		{
				if (!this->has_parameter(name)) {
						this->declare_parameter<T>(name, default_value);
				}

				T value;
				this->get_parameter(name, value);
				return value;
		}

		template<typename MsgT>
		std::shared_ptr<ManagedPublisher<MsgT>> create_rc_publisher(
			const std::string& topic,
			const rclcpp::QoS & qos = rclcpp::QoS(10),
			bool required = false
		)
		{
				auto pub = std::make_shared<ManagedPublisher<MsgT>>(this, topic, qos, required);
				registered_rc_publishers_.push_back(pub);
				return pub;
		}

		template<typename MsgT>
		std::shared_ptr<ManagedSubscriptor<MsgT>> create_rc_subscription(
			const std::string& topic,
			std::function<void(typename MsgT::SharedPtr)> user_callback,
			const rclcpp::QoS & qos = rclcpp::QoS(10),
			bool required = false
		)
		{
				auto sub = std::make_shared<ManagedSubscriptor<MsgT>>(this, topic, user_callback, qos, required);
				registered_rc_subscriptors_.push_back(sub);
				return sub;
		}

		// Rcomponent user implementations
		virtual CallbackReturn rc_configure() = 0;
    virtual CallbackReturn rc_activate() = 0;
		virtual CallbackReturn rc_deactivate() = 0;
		virtual CallbackReturn rc_cleanup() = 0;
		virtual CallbackReturn rc_shutdown() = 0;
		virtual CallbackReturn rc_error() = 0;

		virtual void rc_loop() = 0;

	private:

		double loop_frequency_{1.0};

		rclcpp::TimerBase::SharedPtr timer_;
		std::shared_ptr<StateManager> rmanager_;

	  // Lifecycle callbacks
		CallbackReturn on_configure(const rclcpp_lifecycle::State &);
		CallbackReturn on_activate(const rclcpp_lifecycle::State &);
		CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
		CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
		CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);
		CallbackReturn on_error(const rclcpp_lifecycle::State &);
		
		std::vector<std::shared_ptr<ManagedPublisherInterface>> registered_rc_publishers_;
		std::vector<std::shared_ptr<ManagedSubscriptorInterface>> registered_rc_subscriptors_;

};
}