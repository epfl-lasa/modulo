#include "modulo_core/Modulator.hpp"

namespace ModuloCore
{
	Modulator::Modulator(const std::string & node_name, const std::chrono::milliseconds & period, bool intra_process_comms) : 
	Cell(node_name, period, intra_process_comms)
	{}

	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Modulator::on_configure(const rclcpp_lifecycle::State & state)
	{
		Cell::on_configure(state);
		return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
	}
}