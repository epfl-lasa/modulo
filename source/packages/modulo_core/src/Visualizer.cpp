#include "modulo_core/Visualizer.hpp"

namespace Modulo
{
	namespace Visualizers
	{
		Visualizer::~Visualizer()
		{
			this->on_shutdown();
		}

		void Visualizer::on_configure()
		{}

		void Visualizer::on_activate()
		{}

		void Visualizer::on_deactivate()
		{}

		void Visualizer::on_cleanup()
		{}

		void Visualizer::on_shutdown()
		{}
	}
}