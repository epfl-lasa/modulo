#include "modulo_core/MotionGenerator.hpp"

namespace Modulo
{
	namespace MotionGenerators
	{
		MotionGenerator::~MotionGenerator()
		{
			this->on_shutdown();
		}

		bool MotionGenerator::on_configure()
		{
			return true;
		}

		bool MotionGenerator::on_activate()
		{
			return true;
		}

		bool MotionGenerator::on_deactivate()
		{
			return true;
		}

		bool MotionGenerator::on_cleanup()
		{
			return true;
		}

		bool MotionGenerator::on_shutdown()
		{
			return true;
		}
	}
}