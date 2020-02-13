#include "modulo_core/Monitor.hpp"

namespace Modulo
{
	namespace Monitors
	{
		Monitor::Monitor(const std::list<std::string>& monitored_node, const std::string & node_name, const std::chrono::milliseconds & period, bool intra_process_comms) : 
		Cell(node_name, period, intra_process_comms), monitored_node_(monitored_node)
		{}

		Monitor::~Monitor()
		{
			this->on_shutdown();
		}

		void Monitor::on_configure()
		{
			for (auto& name : this->monitored_node_) this->add_client<lifecycle_msgs::srv::GetState>(name + "/get_state", this->get_period());
		}

		void Monitor::on_activate()
		{}

		void Monitor::on_deactivate()
		{}

		void Monitor::on_cleanup()
		{}

		void Monitor::on_shutdown()
		{}

		void Monitor::step()
		{
			for (auto& name : this->monitored_node_)
			{
				auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
				auto response = this->send_blocking_request<lifecycle_msgs::srv::GetState>(name + "/get_state", request);
				RCLCPP_INFO(get_logger(), "Node %s status is %s", name.c_str(), response.get()->current_state.label.c_str());
			}
		}
	}
}