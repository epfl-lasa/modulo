#include "modulo_core/MotionGenerator.hpp"

namespace Modulo
{
	namespace MotionGenerators
	{
		MotionGenerator::MotionGenerator(const std::string & node_name, const std::chrono::milliseconds & period, bool intra_process_comms) : 
		Cell(node_name, period, intra_process_comms)
		{}

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