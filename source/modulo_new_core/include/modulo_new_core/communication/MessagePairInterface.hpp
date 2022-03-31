#pragma once

#include <memory>
#include "modulo_new_core/communication/MessageType.hpp"

namespace modulo_new_core::communication {

// forward declaration of derived MessagePair class
template<typename MsgT, typename DataT>
class MessagePair;

class MessagePairInterface : public std::enable_shared_from_this<MessagePairInterface> {
public:
  explicit MessagePairInterface(MessageType type);

  virtual ~MessagePairInterface() = default;

  MessagePairInterface(const MessagePairInterface& message_pair) = default;

  template<typename MsgT, typename DataT>
  std::shared_ptr<MessagePair<MsgT, DataT>> get_message_pair(bool validate_pointer = true);

  template<typename MsgT, typename DataT>
  MsgT get_message_pair_message();

  MessageType get_type() const;

private:
  MessageType type_;
};

template<typename MsgT, typename DataT>
inline std::shared_ptr<MessagePair<MsgT, DataT>> MessagePairInterface::get_message_pair(bool validate_pointer) {
  std::shared_ptr<MessagePair<MsgT, DataT>> message_pair_ptr;
  try {
    message_pair_ptr = std::dynamic_pointer_cast<MessagePair<MsgT, DataT>>(this->shared_from_this());
  } catch (const std::exception& ex) {
    if (validate_pointer) {
      // TODO do we need a name for the MessagePairInterface?
      throw std::runtime_error("MessagePair interface \"test\" is not managed by a valid pointer");
    }
  }
  if (message_pair_ptr == nullptr && validate_pointer) {
    throw std::runtime_error("Unable to cast MessagePair interface");
  }
  return message_pair_ptr;
}

template<typename MsgT, typename DataT>
inline MsgT MessagePairInterface::get_message_pair_message() {
  return this->template get_message_pair<MsgT, DataT>()->get_message();
}

inline MessageType MessagePairInterface::get_type() const {
  return this->type_;
}

}// namespace modulo_new_core::communication
