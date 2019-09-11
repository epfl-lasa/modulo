#include "modulo_core/RobotInterface.hpp"

namespace Modulo
{
	namespace RobotInterfaces
	{
		RobotInterface::RobotInterface(const std::string & node_name, const std::string & robot_name, const std::string & robot_ip, const std::chrono::milliseconds & period, bool intra_process_comms) : 
		Cell(node_name, period, intra_process_comms), robot_name_(robot_name), robot_ip_(robot_ip),
		current_cartesian_state(std::make_shared<StateRepresentation::CartesianState>(robot_name + "/end_effector", robot_name + "/base")),
		current_joint_state(std::make_shared<StateRepresentation::JointState>(robot_name + "/joints")),
		desired_cartesian_state(std::make_shared<StateRepresentation::CartesianState>(robot_name + "/end_effector", robot_name + "/base")),
		desired_joint_state(std::make_shared<StateRepresentation::JointState>(robot_name + "/joints"))
		{}

		RobotInterface::~RobotInterface()
		{
			this->on_shutdown();
		}

		void RobotInterface::on_configure()
		{}

		void RobotInterface::on_activate()
		{}

		void RobotInterface::on_deactivate()
		{}

		void RobotInterface::on_cleanup()
		{}

		void RobotInterface::on_shutdown()
		{}
	}
}