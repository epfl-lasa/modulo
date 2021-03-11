#include "modulo_core/Communication/CommunicationHandler.hpp"

namespace modulo::core::communication {
CommunicationHandler::CommunicationHandler(const CommunicationType& type,
                                           const std::shared_ptr<std::mutex>& mutex) : type_(type),
                                                                                       mutex_(mutex) {}
}// namespace modulo::core::communication
