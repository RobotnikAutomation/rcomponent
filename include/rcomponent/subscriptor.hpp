#pragma once

#include <rclcpp/subscription_base.hpp>

class ManagedSubscriptorInterface
{
	public:
		virtual void activate() = 0;
		virtual void deactivate() = 0;
		virtual void clear() = 0;
		virtual double time_since_last_activity() = 0;
		virtual rclcpp::SubscriptionBase::SharedPtr get() = 0;
};

template<typename MessageT>
class ManagedSubscriptor : public ManagedSubscriptorInterface
{
	public:

		using SharedPtr = std::shared_ptr<ManagedSubscriptor<MessageT>>; 

		// This class is meant to be created in rc_configure, so this constructor
		// creates the interfaces
		ManagedSubscriptor(rclcpp_lifecycle::LifecycleNode* node,
											const std::string & topic_name,
											std::function<void(typename MessageT::SharedPtr)> user_callback,
											const rclcpp::QoS & qos,
											bool required
											)
			: node_(node),
				topic_name_(topic_name),
				user_callback_(user_callback),
				required_(required)
		{
			sub_ = node_->create_subscription<MessageT>(topic_name_, qos, [this](typename MessageT::SharedPtr msg) {
				managed_callback(msg);
			});

		}

		void activate() override
		{
			last_msg_time_ = node_->now();
			process_user_callback_ = true;
		}

		void deactivate() override
		{
			process_user_callback_ = false;
		}

		void clear() override 
		{
			sub_.reset();
		}

		double time_since_last_activity() override
		{
			if(required_)
				return (node_->now() - last_msg_time_).seconds();
			
			return 0;
		}

		rclcpp::SubscriptionBase::SharedPtr get() override
		{
    	return sub_;
		}
	
	private:

		rclcpp_lifecycle::LifecycleNode* node_;
		std::string topic_name_;
		typename rclcpp::Subscription<MessageT>::SharedPtr sub_;
		bool required_;
		std::function<void(typename MessageT::SharedPtr)> user_callback_;
		bool process_user_callback_ = false;
		rclcpp::Time last_msg_time_;

		void managed_callback(typename MessageT::SharedPtr msg)
		{
			last_msg_time_ = node_->now(); 
			if (process_user_callback_) {
				user_callback_(msg);
			}
		}

};

namespace rcomponent {
    template<typename MessageT>
    using Subscriptor = ManagedSubscriptor<MessageT>;
}