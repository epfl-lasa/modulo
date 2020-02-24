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

namespace Modulo
{
	namespace RobotInterfaces
	{
		/// Enumeration of the possible controllers
		enum class Controller 
		{
			CARTESIAN_POSE, /// Caresian pose controller
			CARTESIAN_TWIST, /// Caresian twist controller
			CARTESIAN_WRENCH, /// Caresian wrench controller
			JOINT_POSITIONS, /// Joint positions controller
			JOINT_VELOCITIES, /// Joint velocities controller
			JOINT_TORQUES /// Joint torques controller
		};

		class RobotInterface: public Core::Cell 
		{
		protected:
			std::string robot_name_; ///< name of the robot to control
			std::string robot_ip_; ///< ip adress of the robot to control
			std::shared_ptr<StateRepresentation::CartesianState> current_cartesian_state; ///< pointer to the current cartesian state
			std::shared_ptr<StateRepresentation::JointState> current_joint_state; ///< pointer to the current joint state
			std::shared_ptr<StateRepresentation::JacobianMatrix> jacobian; ///< pointer to the jacobian
			std::shared_ptr<StateRepresentation::CartesianState> desired_cartesian_state; ///< pointer to the desired cartesian state
			std::shared_ptr<StateRepresentation::JointState> desired_joint_state; ///< pointer to the desired joint state
			Controller active_controller; ///< the current active controller type 

		public:
			/**
			 * @brief Constructor for the RobotInterface class
			 * @param node_name name of the ROS node
			 * @param robot_ip ip adress of the robot to control
			 * @param period rate used by each publisher of the class
			 */
			template <typename DurationT>
			explicit RobotInterface(const std::string& node_name, const std::string& robot_name, const std::string& robot_ip, const std::chrono::duration<int64_t, DurationT>& period, bool intra_process_comms=false);

			/**
			 * @brief Destructor
			 */
			~RobotInterface();

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
			 * @brief This function is called time the configure call 
			 * is made from the lifecycle server. It is used to
			 * define behavior such as connecting to a database 
			 * or resetting an history buffer. After being 
			 * configured the node can be activated.
			 */
			virtual void on_configure();

			/**
			 * @brief This function is called time the activate call 
			 * is made from the lifecycle server. It activates publishing
			 * and subsciptions and can be extended to start a recording
			 * or replay.
			 */
			virtual void on_activate();

			/**
			 * @brief This function is called time the deactivate call 
			 * is made from the lifecycle server. It deactivates publishing
			 * and subsciptions and can be extended to stop a recording
			 * or a replay.
			 */
			virtual void on_deactivate();

			/**
			 * @brief This function is called time the cleanup call 
			 * is made from the lifecycle server. It cleans the node
			 * and can be extended to close connections to a database
			 * or delete pointers. After cleanup a new configure call
			 * can be made.
			 */
			virtual void on_cleanup();

			/**
			 * @brief This function is called time the shutdown call 
			 * is made from the lifecycle server. It terminates the node.
			 * Each elements needed to be cleaned before termination should
			 * be here.
			 */
			virtual void on_shutdown();

			/**
			 * @brief Function computing one step of calculation. It is called periodically in the run function.
			 */
			virtual void step() = 0;
		};

		template <typename DurationT>
		RobotInterface::RobotInterface(const std::string& node_name, const std::string& robot_name, const std::string& robot_ip, const std::chrono::duration<int64_t, DurationT>& period, bool intra_process_comms): 
		Cell(node_name, period, intra_process_comms), robot_name_(robot_name), robot_ip_(robot_ip),
		current_cartesian_state(std::make_shared<StateRepresentation::CartesianState>(robot_name + "_end_effector", robot_name + "_base")),
		current_joint_state(std::make_shared<StateRepresentation::JointState>(robot_name)),
		jacobian(std::make_shared<StateRepresentation::JacobianMatrix>(robot_name)),
		desired_cartesian_state(std::make_shared<StateRepresentation::CartesianState>(robot_name + "_end_effector", robot_name + "_base")),
		desired_joint_state(std::make_shared<StateRepresentation::JointState>(robot_name))
		{}

		inline const std::string & RobotInterface::get_robot_ip() const
		{
			return this->robot_ip_;
		}

		inline void RobotInterface::set_robot_ip(const std::string & robot_ip)
		{
			this->robot_ip_ = robot_ip;
		}
	}
}
#endif