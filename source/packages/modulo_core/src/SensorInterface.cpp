#include "modulo_core/SensorInterface.hpp"

namespace Modulo
{
	namespace SensorInterfaces
	{
		SensorInterface::~SensorInterface()
		{
			this->on_shutdown();
		}

		void SensorInterface::on_configure()
		{}

		void SensorInterface::on_activate()
		{}

		void SensorInterface::on_deactivate()
		{}

		void SensorInterface::on_cleanup()
		{}

		void SensorInterface::on_shutdown()
		{}
	}
}