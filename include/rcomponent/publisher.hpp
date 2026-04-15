#pragma once

#include <rclcpp/publisher_base.hpp>

class ManagedPublisherInterface
{
	public:
		virtual void activate() = 0;
		virtual void deactivate() = 0;
		virtual void clear() = 0;
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
                     const rclcpp::QoS & qos = rclcpp::QoS(10))
        : node_(node),
					topic_name_(topic_name)
    {
			pub_ = node_->create_publisher<MessageT>(topic_name_, 10);
    }

		void activate() override
		{
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
    typename rclcpp_lifecycle::LifecyclePublisher<MessageT>::SharedPtr pub_;
};

namespace rcomponent {
    template<typename MessageT>
    using Publisher = ManagedPublisher<MessageT>;
}