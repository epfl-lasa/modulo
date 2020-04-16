#include "modulo_core/Visualizer.hpp"

namespace Modulo
{
	namespace Visualizers
	{
		Visualizer::~Visualizer()
		{
			this->on_shutdown();
		}

		bool Visualizer::on_configure()
		{
			return true;
		}

		bool Visualizer::on_activate()
		{
			return true;
		}

		bool Visualizer::on_deactivate()
		{
			return true;
		}

		bool Visualizer::on_cleanup()
		{
			return true;
		}

		bool Visualizer::on_shutdown()
		{
			return true;
		}
	}
}