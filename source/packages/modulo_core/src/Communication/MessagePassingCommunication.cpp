#include "modulo_core/Communication/MessagePassingCommunication.hpp"

namespace Modulo
{
	namespace Core
	{
		namespace Communication
		{
			MessagePassingCommunication::MessagePassingCommunication(const CommunicationType& type, const std::shared_ptr<StateRepresentation::State>& recipient, const std::chrono::milliseconds& timeout, const std::shared_ptr<std::mutex>& mutex):
			CommunicationHandler(type, timeout, mutex),
			recipient_(recipient)
			{}
		}
	}
}