#include "modulo_core/Communication/CommunicationHandler.hpp"

namespace Modulo
{
	namespace Core
	{
		namespace Communication
		{
			CommunicationHandler::CommunicationHandler(const CommunicationType& type, const std::string& channel, const std::shared_ptr<StateRepresentation::State>& recipient, const std::chrono::milliseconds& timeout, const std::shared_ptr<rclcpp::Clock>& clock, const std::shared_ptr<std::mutex>& mutex):
			type_(type), channel_(channel), recipient_(recipient), timeout_(timeout), clock_(clock), mutex_(mutex)
			{}
		}
	}
}