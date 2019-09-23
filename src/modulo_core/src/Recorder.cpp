#include "modulo_core/Recorder.hpp"

namespace Modulo
{
	namespace Recorders
	{
		Recorder::Recorder(const std::string & node_name, const std::chrono::milliseconds & period, bool intra_process_comms) : 
		Cell(node_name, period, intra_process_comms)
		{}

		Recorder::~Recorder()
		{}

		void Recorder::on_configure()
		{}

		void Recorder::on_activate()
		{
			this->start_time = std::chrono::system_clock::now();
		}

		void Recorder::on_deactivate()
		{
			this->end_time = std::chrono::system_clock::now();
		}

		void Recorder::on_cleanup()
		{}

		void Recorder::on_shutdown()
		{}

		void Recorder::step()
		{
			for (auto &h : this->get_handlers())
			{
				if(h.second->get_type() == "subscription")
				{
					if(!this->record(h.second->get_recipient())) RCLCPP_ERROR(this->get_logger(), "Unable to record " + h.second->get_recipient().get_name());
				}
			}
		}

		bool Recorder::record(const StateRepresentation::State& state) const
		{
			if(typeid(state) == typeid(StateRepresentation::CartesianState))
			{
				return record(static_cast<const StateRepresentation::CartesianState&>(state));
			}
			RCLCPP_ERROR(this->get_logger(), "Recording function for " + state.get_name() + " not defined for this type of state");
			return false;
		}

		bool Recorder::record(const StateRepresentation::CartesianState& state) const
		{
			RCLCPP_WARN(this->get_logger(), "Trying to record " + state.get_name() + " from the base class");
			return false;
		}
	}
}