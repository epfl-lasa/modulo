#include "modulo_core/MotionGenerator.hpp"

namespace Modulo
{
	namespace MotionGenerators
	{
		MotionGenerator::~MotionGenerator()
		{
			this->on_shutdown();
		}

		void MotionGenerator::on_configure()
		{}

		void MotionGenerator::on_activate()
		{}

		void MotionGenerator::on_deactivate()
		{}

		void MotionGenerator::on_cleanup()
		{}

		void MotionGenerator::on_shutdown()
		{}
	}
}