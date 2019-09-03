#include "modulo_core/MotionGenerator.hpp"

namespace ModuloCore
{
	MotionGenerator::MotionGenerator(const std::string & node_name, const std::chrono::milliseconds & period, bool intra_process_comms) : 
	Cell(node_name, period, intra_process_comms)
	{}

	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MotionGenerator::on_configure(const rclcpp_lifecycle::State & state)
	{
		Cell::on_configure(state);
		return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
	}

	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MotionGenerator::on_activate(const rclcpp_lifecycle::State & state)
	{
		Cell::on_activate(state);
		return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
	}

	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MotionGenerator::on_deactivate(const rclcpp_lifecycle::State & state)
	{
		Cell::on_deactivate(state);
		return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
	}

	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MotionGenerator::on_cleanup(const rclcpp_lifecycle::State & state)
	{
		Cell::on_cleanup(state);
		return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
	}

	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MotionGenerator::on_shutdown(const rclcpp_lifecycle::State & state)
	{
		Cell::on_shutdown(state);
		return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
	}
}