#include "modulo_core/communication/message_passing/MessagePassingHandler.hpp"

namespace modulo::core::communication {
MessagePassingHandler::MessagePassingHandler(const CommunicationType& type) : CommunicationHandler(type),
                                                                              asynchronous_(false) {}

MessagePassingHandler::MessagePassingHandler(const CommunicationType& type,
                                             const std::shared_ptr<state_representation::State>& recipient) : CommunicationHandler(type),
                                                                                                              recipient_(recipient),
                                                                                                              asynchronous_(true) {}
}// namespace modulo::core::communication
