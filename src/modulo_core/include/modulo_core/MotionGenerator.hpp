/**
 * @class MotionGenerator
 * @brief Abstract class to define a MotionGenerator
 * @author Baptiste Busch
 * @date 2019/02/14
 *
 * A MotionGenerator generate desired motion from a current state.
 */

#ifndef MODULO_MOTION_GENERATOR_H_
#define MODULO_MOTION_GENERATOR_H_

#include "modulo_core/Cell.hpp"

namespace ModuloCore
{
	class MotionGenerator: public Cell 
	{
	public:
		/**
		 * @brief Constructor for the MotionGenerator class
		 * @param node_name name of the ROS node
		 * @param period rate used by each publisher of the class
		 */
		explicit MotionGenerator(const std::string & node_name, const std::chrono::milliseconds & period, bool intra_process_comms = false);

		/** 
		 * @brief Abrtract function from the lifecycle interface
		 *
		 * This function is used each time the configure call 
		 * is made from the lifecycle server. It defines all 
		 * the publishers and subscriptions of the node.
		 */
		virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State & state);

		/**
		 * @brief Transition callback for state activating
		 *
		 * on_activate callback is being called when the lifecycle node
		 * enters the "activating" state.
		 * Depending on the return value of this function, the state machine
		 * either invokes a transition to the "active" state or stays
		 * in "inactive".
		 * TRANSITION_CALLBACK_SUCCESS transitions to "active"
		 * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
		 * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
		 */
		virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state);

		/**
		 * @brief Transition callback for state deactivating
		 *
		 * on_deactivate callback is being called when the lifecycle node
		 * enters the "deactivating" state.
		 * Depending on the return value of this function, the state machine
		 * either invokes a transition to the "inactive" state or stays
		 * in "active".
		 * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
		 * TRANSITION_CALLBACK_FAILURE transitions to "active"
		 * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
		 */
		virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state);

		/**
		 * @brief Transition callback for state cleaningup
		 *
		 * on_cleanup callback is being called when the lifecycle node
		 * enters the "cleaningup" state.
		 * Depending on the return value of this function, the state machine
		 * either invokes a transition to the "unconfigured" state or stays
		 * in "inactive".
		 * TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
		 * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
		 * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
		 */
		virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state);

		/**
		 * @brief Transition callback for state shutting down
		 *
		 * on_shutdown callback is being called when the lifecycle node
		 * enters the "shuttingdown" state.
		 * Depending on the return value of this function, the state machine
		 * either invokes a transition to the "finalized" state or stays
		 * in its current state.
		 * TRANSITION_CALLBACK_SUCCESS transitions to "finalized"
		 * TRANSITION_CALLBACK_FAILURE transitions to current state
		 * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
		 */
		virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);

		/**
		 * @brief Function computing one step of calculation. It is called periodically in the run function.
		 */
		virtual void step()=0;
	};
}
#endif