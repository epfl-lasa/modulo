#pragma once

#include <memory>

#include "modulo_new_core/communication/MessageType.hpp"
#include "modulo_new_core/exceptions/InvalidPointerCastException.hpp"
#include "modulo_new_core/exceptions/InvalidPointerException.hpp"

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
  [[nodiscard]] std::shared_ptr<MessagePair<MsgT, DataT>> get_message_pair(bool validate_pointer = true);

  template<typename MsgT, typename DataT>
  [[nodiscard]] MsgT write();

  template<typename MsgT, typename DataT>
  void read(const MsgT& message);

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
      throw exceptions::InvalidPointerException("Message pair interface is not managed by a valid pointer");
    }
  }
  if (message_pair_ptr == nullptr && validate_pointer) {
    throw exceptions::InvalidPointerCastException(
        "Unable to cast message pair interface to a message pair pointer of requested type"
    );
  }
  return message_pair_ptr;
}

template<typename MsgT, typename DataT>
inline MsgT MessagePairInterface::write() {
  return this->template get_message_pair<MsgT, DataT>()->write_message();
}

template<typename MsgT, typename DataT>
inline void MessagePairInterface::read(const MsgT& message) {
  this->template get_message_pair<MsgT, DataT>()->read_message(message);
}

}// namespace modulo_new_core::communication
