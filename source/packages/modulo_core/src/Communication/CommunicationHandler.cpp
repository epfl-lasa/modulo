#include "modulo_core/Communication/CommunicationHandler.hpp"

namespace Modulo
{
	namespace Core
	{
		namespace Communication
		{
			CommunicationHandler::CommunicationHandler(const CommunicationType& type, const std::chrono::milliseconds& timeout, const std::shared_ptr<std::mutex>& mutex):
			type_(type), timeout_(timeout), mutex_(mutex)
			{}
		}
	}
}