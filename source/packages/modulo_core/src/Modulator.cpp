#include "modulo_core/Modulator.hpp"

namespace Modulo
{
	namespace Modulators
	{
		Modulator::Modulator(const std::string & node_name, const std::chrono::milliseconds & period, bool intra_process_comms) : 
		Cell(node_name, period, intra_process_comms)
		{}

		Modulator::~Modulator()
		{
			this->on_shutdown();
		}

		void Modulator::on_configure()
		{}

		void Modulator::on_activate()
		{}

		void Modulator::on_deactivate()
		{}

		void Modulator::on_cleanup()
		{}

		void Modulator::on_shutdown()
		{}
	}
}