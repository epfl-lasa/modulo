#include "modulo_core/communication/MessagePairInterface.hpp"

namespace modulo_core::communication {

MessagePairInterface::MessagePairInterface(MessageType type) : type_(type) {}

MessageType MessagePairInterface::get_type() const {
  return this->type_;
}
}// namespace modulo_core::communication
