#include "modulo_core/RobotInterface.hpp"

namespace ModuloRobotInterface
{
	RobotInterface::RobotInterface(const std::string & node_name, const std::string & robot_name, const std::string & robot_ip, const std::chrono::milliseconds & period, bool intra_process_comms) : 
	Cell(node_name, period, intra_process_comms), robot_name_(robot_name), robot_ip_(robot_ip),
	current_cartesian_state(std::make_shared<StateRepresentation::CartesianState>(robot_name + "/end_effector", robot_name + "/base")),
	current_joint_state(std::make_shared<StateRepresentation::JointState>(robot_name + "/joints")),
	desired_cartesian_state(std::make_shared<StateRepresentation::CartesianState>(robot_name + "/end_effector", robot_name + "/base")),
	desired_joint_state(std::make_shared<StateRepresentation::JointState>(robot_name + "/joints"))
	{}

	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotInterface::on_configure(const rclcpp_lifecycle::State & state)
	{
		ModuloCore::Cell::on_configure(state);
		return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
	}

	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotInterface::on_activate(const rclcpp_lifecycle::State & state)
	{
		ModuloCore::Cell::on_activate(state);
		return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
	}

	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotInterface::on_deactivate(const rclcpp_lifecycle::State & state)
	{
		ModuloCore::Cell::on_deactivate(state);
		return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
	}

	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotInterface::on_cleanup(const rclcpp_lifecycle::State & state)
	{
		ModuloCore::Cell::on_cleanup(state);
		return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
	}

	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotInterface::on_shutdown(const rclcpp_lifecycle::State & state)
	{
		ModuloCore::Cell::on_shutdown(state);
		return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
	}
}