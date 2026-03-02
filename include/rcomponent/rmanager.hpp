#pragma once

#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace rcomponent
{	
	/// @brief Lifecycle callback return type.
	using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
	
	/// @brief Shared pointer type for a ROS2 Trigger request.
	using TriggerRequest = std::shared_ptr<std_srvs::srv::Trigger::Request>;
	
	/// @brief Shared pointer type for a ROS2 Trigger response.
	using TriggerResponse = std::shared_ptr<std_srvs::srv::Trigger::Response>;
	
	/// @brief ROS2 Lifecycle state message type.
	using LifecycleState = lifecycle_msgs::msg::State;

	/// @brief ROS2 Lifecycle transition message type.
	using LifecycleTransition = lifecycle_msgs::msg::Transition;

	/**
	 * @class Rmanager
	 * @brief Lifecycle manager for rcomponent.
	 *
	 * This class handles lifecycle transitions for a ROS2 node, providing methods
	 * to handle START and STOP commands and execute the corresponding lifecycle transitions.
	 *
	 * Lifecycle transition policy:
	 * - UNCONFIGURED → INACTIVE → ACTIVE: executed by START
	 * - ACTIVE → INACTIVE → UNCONFIGURED: executed by STOP
	 *
	 * Transitions are logged and errors are reported via RCLCPP_ERROR.
	 * Each transition function returns true on success, false on failure.
	 */
	class Rmanager
	{
		public:

			/**
			 * @brief Construct a new Rmanager object.
			 *
			 * Initializes the lifecycle manager with a given ROS2 lifecycle node.
			 * It creates the START and STOP services, initializes the management thread,
			 * and sets the initial pending command to NONE.
			 *
			 * @param node Shared pointer to the ROS2 lifecycle node to manage.
			 */
			Rmanager(rclcpp_lifecycle::LifecycleNode::SharedPtr node);
			
			/// @brief Destroy the Rmanager object. Automatically cleans up the management thread and services.
			~Rmanager()=default;

		private:

			/// @brief Lifecycle commands for Rmanager.
			///
			/// Used internally to mark the pending command for the node.
			/// - NONE: No command pending
			/// - START: Start node (unconfigured/inactive → active)
			/// - STOP: Stop node (active/inactive → unconfigured)
			enum Rcommand : uint8_t {
					NONE = 0,
					START = 1,
					STOP = 2
			};

			// --------------------------
			// Management loop
			// --------------------------

			/// @brief Thread running the main management loop for processing commands.
    	std::jthread manager_thread_;

			/// @brief Shared pointer to the ROS2 lifecycle node managed by this class.
			rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
			/// @brief Timer used for periodic callbacks (e.g., to check and execute pending commands).
			rclcpp::TimerBase::SharedPtr timer_;

			// --------------------------
			// Services
			// --------------------------
			
			/// @brief ROS2 service to trigger START command.
			rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
			/// @brief ROS2 service to trigger STOP command.
			rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;

			// --------------------------
			// Pending command
			// --------------------------

			/// @brief Currently pending command (NONE, START, STOP).
			uint8_t rcommand_;

			/**
			 * @brief Periodic management loop executed in a separate thread.
			 *
			 * Checks for pending commands and calls `timer_callback()` every 100 ms
			 * until the stop token is requested.
			 *
			 * @param st Stop token to allow graceful thread exit.
			 */
			void management_loop(std::stop_token st);

			/**
			 * @brief Periodic timer callback to handle pending lifecycle commands.
			 *
			 * This function is called periodically by the node's timer. It checks if a 
			 * lifecycle command (`START` or `STOP`) is pending and executes the corresponding 
			 * transitions using `handle_start()` or `handle_stop()`.
			 *
			 * Lifecycle command behavior:
			 * - START: transitions the node from UNCONFIGURED → INACTIVE → ACTIVE
			 * - STOP: transitions the node from ACTIVE → INACTIVE → UNCONFIGURED
			 *
			 * Success and failure of each command are logged. After processing, the pending 
			 * command is cleared.
			 */

			// --------------------------
			// Callbacks
			// --------------------------

			void timer_callback();

			/**
			 * @brief ROS2 Trigger callback for START service.
			 *
			 * Marks the START command as pending and sets the response success flag.
			 *
			 * @param request Unused TriggerRequest
			 * @param response TriggerResponse to indicate request acceptance.
			 */
			void start_callback([[maybe_unused]] TriggerRequest request, TriggerResponse response);
			
			/**
			 * @brief ROS2 Trigger callback for STOP service.
			 *
			 * Marks the STOP command as pending and sets the response success flag.
			 *
			 * @param request Unused TriggerRequest
			 * @param response TriggerResponse to indicate request acceptance.
			 */
			void stop_callback([[maybe_unused]] TriggerRequest request, TriggerResponse response);

			// --------------------------
			// Helper functions
			// --------------------------

			/**
			 * @brief Get human-readable label of a lifecycle transition.
			 *
			 * @param id Transition ID
			 * @return std::string Name of the transition ("configure", "activate", etc.)
			 */
			std::string transition_label(uint8_t id);

			/**
			 * @brief Get human-readable label of a lifecycle state.
			 *
			 * @param id State ID
			 * @return std::string Name of the state ("active", "inactive", etc.)
			 */
			std::string state_label(uint8_t id);

			/**
			 * @brief Get human-readable label of a pending command.
			 *
			 * @param id Command ID
			 * @return std::string Name of the command ("start", "stop", etc.)
			 */
			std::string rcommand_label(uint8_t id);

			/**
			 * @brief Get the target state after a given lifecycle transition.
			 *
			 * @param id Transition ID
			 * @return std::string Name of the resulting state
			 */
			std::string transition_target(uint8_t id);

			// --------------------------
			// Lifecycle transitions
			// --------------------------

			/**
			 * @brief Execute a lifecycle transition safely.
			 *
			 * Triggers the desired transition, checks the callback return value,
			 * validates the new state, and logs errors if the transition fails.
			 *
			 * @param desired_transition Transition to execute
			 * @param rcommand Command that initiated the transition
			 * @return true if transition succeeded, false otherwise
			 */
			bool set_transition(uint8_t desired_transition, uint8_t rcommand);

			/**
			 * @brief Transition from UNCONFIGURED to ACTIVE.
			 *
			 * Performs activation.
			 * @param rcommand Command type (e.g., Rcommand::START)
			 * @return true if successful, false otherwise
			 */
			bool unconfigured_to_active(uint8_t rcommand);

			/**
			 * @brief Transition from INACTIVE to ACTIVE.
			 *
			 * Performs activation.
			 * @param rcommand Command type (e.g., Rcommand::START)
			 * @return true if successful, false otherwise
			 */
			bool inactive_to_active(uint8_t rcommand);

			/**
			 * @brief Transition from ACTIVE to UNCONFIGURED.
			 *
			 * Performs deactivation followed by cleanup.
			 * @param rcommand Command type (e.g., Rcommand::STOP)
			 * @return true if successful, false otherwise
			 */
			bool active_to_unconfigured(uint8_t rcommand);

			/**
			 * @brief Transition from INACTIVE to UNCONFIGURED.
			 *
			 * Performs cleanup.
			 * @param rcommand Command type (e.g., Rcommand::STOP)
			 * @return true if successful, false otherwise
			 */
			bool inactive_to_unconfigured(uint8_t rcommand);

			// --------------------------
			// Command handlers
			// --------------------------

			/**
			 * @brief Handle the START command.
			 *
			 * Executes the lifecycle transitions required to bring the node to ACTIVE.
			 * Supported current states:
			 * - UNCONFIGURED → calls unconfigured_to_active()
			 * - INACTIVE → calls inactive_to_active()
			 *
			 * If the node is in any other state, an error is logged and the command fails.
			 *
			 * @param current_state Current state of the node.
			 * @param rcommand The command to execute (e.g., Rcommand::START).
			 * @return true if the start command succeeded, false otherwise.
			 */
			bool handle_start(uint8_t current_state, uint8_t rcommand);

			/**
			 * @brief Handle the STOP command.
			 *
			 * Executes the lifecycle transitions required to bring the node to UNCONFIGURED.
			 * Supported current states:
			 * - ACTIVE → calls active_to_unconfigured()
			 * - INACTIVE → calls inactive_to_unconfigured()
			 *
			 * If the node is in any other state, an error is logged and the command fails.
			 *
			 * @param current_state Current state of the node.
			 * @param rcommand The command to execute (e.g., Rcommand::STOP).
			 * @return true if the stop command succeeded, false otherwise.
			 */		
			bool handle_stop(uint8_t current_state, uint8_t rcommand);
	};

}