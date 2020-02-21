#include "modulo_core/Controller.hpp"

namespace Modulo
{
	namespace Controllers
	{
		Controller::~Controller()
		{
			this->on_shutdown();
		}

		void Controller::on_configure()
		{}

		void Controller::on_activate()
		{}

		void Controller::on_deactivate()
		{}

		void Controller::on_cleanup()
		{}

		void Controller::on_shutdown()
		{}
	}
}