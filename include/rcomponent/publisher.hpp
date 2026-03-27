#pragma once

class ManagedPublisherInterface
{
	public:
		std::string topic_name;
		virtual void activate() = 0;
		virtual void deactivate() = 0;
		virtual void clear() = 0;

};

template<typename MessageT>
class ManagedPublisher : public ManagedPublisherInterface
{
public:
		
		using SharedPtr = std::shared_ptr<ManagedPublisher<MessageT>>; 

    ManagedPublisher(rclcpp_lifecycle::LifecycleNode* node,
                     const std::string & topic_name,
                     const rclcpp::QoS & qos = rclcpp::QoS(10))
        : node_(node),
					topic_name_(topic_name)
    {	
			// Dado que esta clase debe crear en rc_configure como 
			// rclcpp::Publisher, esto es configure
			pub_ = node_->create_publisher<MessageT>(topic_name_, 10);
    }

		void activate()
		{
			pub_->on_activate();
		}

		void deactivate()
		{
			pub_->on_deactivate();
		}

    void clear() {
      pub_.reset();
    }

    void publish(const MessageT & msg) {
        if (pub_) pub_->publish(msg);
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