#include "modulo_core/RobotInterface.hpp"

namespace Modulo
{
	namespace RobotInterfaces
	{
		RobotInterface::~RobotInterface()
		{
			this->on_shutdown();
		}

		void RobotInterface::on_configure()
		{}

		void RobotInterface::on_activate()
		{}

		void RobotInterface::on_deactivate()
		{}

		void RobotInterface::on_cleanup()
		{}

		void RobotInterface::on_shutdown()
		{}
	}
}