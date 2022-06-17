#include "modulo_new_core/communication/MessagePairInterface.hpp"

namespace modulo_new_core::communication {

MessagePairInterface::MessagePairInterface(MessageType type) : type_(type) {}

MessageType MessagePairInterface::get_type() const {
  return this->type_;
}
}// namespace modulo_new_core::communication
