#include "modulo_core/Monitor.hpp"
#include "modulo_core/Exceptions/CommunicationTimeoutException.hpp"

namespace Modulo
{
	namespace Monitors
	{
		Monitor::~Monitor()
		{
			this->on_shutdown();
		}

		void Monitor::on_configure()
		{
			for (auto& name : this->monitored_node_) this->add_client<lifecycle_msgs::srv::GetState>(name + "/get_state", std::chrono::milliseconds(100));
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
				try
				{
					auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
					auto response = this->send_blocking_request<lifecycle_msgs::srv::GetState>(name + "/get_state", request);
					RCLCPP_INFO(get_logger(), "Node %s status is %s", name.c_str(), response->current_state.label.c_str());
				}
				catch (Exceptions::CommunicationTimeoutException& e)
				{
					RCLCPP_ERROR(get_logger(), e.what());
				}
			}
			RCLCPP_INFO(get_logger(), "----------------------"); 
		}
	}
}