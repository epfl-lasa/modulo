#include "modulo_core/Recorder.hpp"

namespace Modulo
{
	namespace Recorders
	{
		Recorder::Recorder(const std::string & node_name, const std::chrono::milliseconds & period, bool intra_process_comms) : 
		Cell(node_name, period, intra_process_comms)
		{}

		void Recorder::on_activate()
		{
			this->start_time = std::chrono::system_clock::now();
		}

		void Recorder::on_deactivate()
		{
			this->end_time = std::chrono::system_clock::now();
		}

		void Recorder::step()
		{
			for (auto &h : this->get_handlers())
			{
				this->record(h.second->get_recipient());
			}
		}

		bool Recorder::record(const StateRepresentation::State& state) const
		{
			if(typeid(state) == typeid(StateRepresentation::CartesianState))
			{
				return record(static_cast<const StateRepresentation::CartesianState&>(state));
			}
			return false;
		}
	}
}