/**
 * @class RobotInterface
 * @brief Abstract class to define a RobotInterface
 * @author Baptiste Busch
 * @date 2019/06/12
 *
 * A RobotInterface is a class to interface with a robotic hardware.
 * It provides the API for the command that can be sent to the system
 * and returns the current state of the robot.
 */

#ifndef MODULOCORE_ROBOTINTERFACE_H_
#define MODULOCORE_ROBOTINTERFACE_H_

#include "modulo_core/Cell.hpp"

namespace ModuloRobotInterface
{
	class RobotInterface: public ModuloCore::Cell 
	{
	protected:
		std::string robot_name_;
		std::string robot_ip_; ///< ip adress of the robot to control
		std::shared_ptr<StateRepresentation::CartesianState> current_cartesian_state;
		std::shared_ptr<StateRepresentation::JointState> current_joint_state;
		std::shared_ptr<StateRepresentation::CartesianState> desired_cartesian_state;
		std::shared_ptr<StateRepresentation::JointState> desired_joint_state;

	public:
		/**
		 * @brief Constructor for the RobotInterface class
		 * @param node_name name of the ROS node
		 * @param robot_ip ip adress of the robot to control
		 * @param period rate used by each publisher of the class
		 */
		explicit RobotInterface(const std::string & node_name, const std::string & robot_name, const std::string & robot_ip, const std::chrono::milliseconds & period, bool intra_process_comms = false);

		/**
		 * @brief Getter of the robot_ip_ variable
		 * @return Reference to the robot_ip_ variable
		 */
		const std::string & get_robot_ip() const;

		/**
		 * @brief Setter of the robot_ip_ variable
		 * @param robot_ip the new ip adress
		 */
		void set_robot_ip(const std::string & robot_ip);

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

	inline const std::string & RobotInterface::get_robot_ip() const
	{
		return this->robot_ip_;
	}

	inline void RobotInterface::set_robot_ip(const std::string & robot_ip)
	{
		this->robot_ip_ = robot_ip;
	}
}
#endif