#pragma once

#include <rclcpp/publisher_base.hpp>

class ManagedPublisherInterface
{
	public:
		virtual void activate() = 0;
		virtual void deactivate() = 0;
		virtual void clear() = 0;
		virtual double time_since_last_activity() = 0;
		virtual rclcpp::PublisherBase::SharedPtr get() = 0;
};

template<typename MessageT>
class ManagedPublisher : public ManagedPublisherInterface
{
public:
		
		using SharedPtr = std::shared_ptr<ManagedPublisher<MessageT>>; 

		// This class is meant to be created in rc_configure, so this constructor
		// creates the interfaces
    ManagedPublisher(rclcpp_lifecycle::LifecycleNode* node,
                     const std::string & topic_name,
                     const rclcpp::QoS & qos,
										 bool required)
        : node_(node),
					topic_name_(topic_name),
					required_(required)
    {
			pub_ = node_->create_publisher<MessageT>(topic_name_, qos);
    }

		void activate() override
		{
			last_msg_time_ = node_->now();
			pub_->on_activate();
		}

		void deactivate() override
		{
			pub_->on_deactivate();
		}

    void clear() override
		{
      pub_.reset();
    }

		double time_since_last_activity() override
		{
			const auto subscription_count =
					pub_->get_subscription_count() +
					pub_->get_intra_process_subscription_count();

			if (subscription_count > 0)
			{
      	last_msg_time_ = node_->now();
			}

			if(required_)
				return (node_->now() - last_msg_time_).seconds();
			
			return 0;
		}
		
    void publish(const MessageT & msg)
		{
        if (pub_) pub_->publish(msg);
    }

		rclcpp::PublisherBase::SharedPtr get() override
		{
    	return pub_;
		}

private:
    rclcpp_lifecycle::LifecycleNode* node_;
		std::string topic_name_;
		bool required_;
    typename rclcpp_lifecycle::LifecyclePublisher<MessageT>::SharedPtr pub_;
		mutable rclcpp::Time last_msg_time_;
};

namespace rcomponent {
    template<typename MessageT>
    using Publisher = ManagedPublisher<MessageT>;
}