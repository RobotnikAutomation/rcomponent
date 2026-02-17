#pragma once

#include <rclcpp/rclcpp.hpp>

class Rcomponent : public rclcpp::Node
{

	public:

		Rcomponent() = delete;
		explicit Rcomponent(const std::string& node_name);
		void printer();

	private:

};