#include "modulo_core/Communication/MessagePassing/MessagePassingHandler.hpp"

namespace modulo::core::communication {
MessagePassingHandler::MessagePassingHandler(const CommunicationType& type,
                                             const std::shared_ptr<std::mutex>& mutex) : CommunicationHandler(type, mutex),
                                                                                         asynchronous_(false) {}

MessagePassingHandler::MessagePassingHandler(const CommunicationType& type,
                                             const std::shared_ptr<StateRepresentation::State>& recipient,
                                             const std::shared_ptr<std::mutex>& mutex) : CommunicationHandler(type, mutex),
                                                                                         recipient_(recipient),
                                                                                         asynchronous_(true) {}
}// namespace modulo::core::communication
