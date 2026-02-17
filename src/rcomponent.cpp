#include "rcomponent/rcomponent.hpp"

Rcomponent::Rcomponent(const std::string& node_name)
	:	rclcpp::Node(node_name)
{
}

void Rcomponent::printer()
{
	RCLCPP_INFO(get_logger(), "I am rcomponent printer");

}
