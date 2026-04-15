#pragma once

#include <rclcpp/subscription_base.hpp>

class ManagedSubscriptorInterface
{
	public:
		virtual void activate() = 0;
		virtual void deactivate() = 0;
		virtual void clear() = 0;
		virtual bool healthcheck(double timeout_sec = 1.0) const = 0;
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
											std::function<void(typename MessageT::SharedPtr)> user_callback
											)
			: node_(node),
				topic_name_(topic_name),
				user_callback_(user_callback)
		{
			sub_ = node_->create_subscription<MessageT>(topic_name_, 10, [this](typename MessageT::SharedPtr msg) {
				managed_callback(msg);
			});

			last_msg_time_ = node_->now();
		}

		void activate() override
		{
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

		bool healthcheck(double timeout_sec = 1.0) const override
		{
        if (!sub_ || !node_) return false;
        auto now = node_->now();
        return (now - last_msg_time_).seconds() < timeout_sec;
    }

		rclcpp::SubscriptionBase::SharedPtr get() override
		{
    	return sub_;
		}
	
	private:

		rclcpp_lifecycle::LifecycleNode* node_;
		std::string topic_name_;
		typename rclcpp::Subscription<MessageT>::SharedPtr sub_;
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