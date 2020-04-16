#include "modulo_core/Modulator.hpp"

namespace Modulo
{
	namespace Modulators
	{
		Modulator::~Modulator()
		{
			this->on_shutdown();
		}

		bool Modulator::on_configure()
		{
			return true;
		}

		bool Modulator::on_activate()
		{
			return true;
		}

		bool Modulator::on_deactivate()
		{
			return true;
		}

		bool Modulator::on_cleanup()
		{
			return true;
		}

		bool Modulator::on_shutdown()
		{
			return true;
		}
	}
}