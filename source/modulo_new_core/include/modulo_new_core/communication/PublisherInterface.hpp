#pragma once

#include <memory>

#include "modulo_new_core/communication/MessagePair.hpp"
#include "modulo_new_core/communication/PublisherType.hpp"
#include "modulo_new_core/exceptions/InvalidPointerCastException.hpp"
#include "modulo_new_core/exceptions/InvalidPointerException.hpp"

namespace modulo_new_core::communication {

// forward declaration of derived Publisher class
template<typename PubT, typename MsgT>
class PublisherHandler;

class PublisherInterface : public std::enable_shared_from_this<PublisherInterface> {
public:
  explicit PublisherInterface(PublisherType type, std::shared_ptr<MessagePairInterface> message_pair = nullptr);

  PublisherInterface(const PublisherInterface& publisher) = default;

  virtual ~PublisherInterface() = default;

  template<typename PubT, typename MsgT>
  std::shared_ptr<PublisherHandler<PubT, MsgT>> get_handler(bool validate_pointer = true);

  void activate();

  void deactivate();

  void publish();

  [[nodiscard]] std::shared_ptr<MessagePairInterface> get_message_pair() const;

  void set_message_pair(const std::shared_ptr<MessagePairInterface>& message_pair);

  PublisherType get_type() const;

private:
  template<typename MsgT>
  void publish(const MsgT& message);

  PublisherType type_;
  std::shared_ptr<MessagePairInterface> message_pair_;
};

template<typename PubT, typename MsgT>
inline std::shared_ptr<PublisherHandler<PubT, MsgT>> PublisherInterface::get_handler(bool validate_pointer) {
  std::shared_ptr<PublisherHandler<PubT, MsgT>> publisher_ptr;
  try {
    publisher_ptr = std::dynamic_pointer_cast<PublisherHandler<PubT, MsgT>>(this->shared_from_this());
  } catch (const std::exception& ex) {
    if (validate_pointer) {
      throw exceptions::InvalidPointerException("Publisher interface is not managed by a valid pointer");
    }
  }
  if (publisher_ptr == nullptr && validate_pointer) {
    throw exceptions::InvalidPointerCastException(
        "Unable to cast publisher interface to a publisher pointer of requested type"
    );
  }
  return publisher_ptr;
}

}// namespace modulo_new_core::communication
