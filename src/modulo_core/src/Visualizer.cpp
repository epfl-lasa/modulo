#include "modulo_core/Visualizer.hpp"

namespace ModuloCore
{
	Visualizer::Visualizer(const std::string & node_name, const std::chrono::milliseconds & period, bool intra_process_comms) : 
	Cell(node_name, period, intra_process_comms)
	{}

	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Visualizer::on_configure(const rclcpp_lifecycle::State & state)
	{
		Cell::on_configure(state);
		return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
	}
}