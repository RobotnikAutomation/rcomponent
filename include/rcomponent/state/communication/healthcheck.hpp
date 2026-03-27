#pragma

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "rcomponent/types.hpp"
#include "rcomponent/utils/log_macros.hpp"

class HealthCheck{

	public:
		HealthCheck(rclcpp_lifecycle::LifecycleNode::SharedPtr node, rclcpp::Logger logger);
		~HealthCheck()=default;

	private:

	

};