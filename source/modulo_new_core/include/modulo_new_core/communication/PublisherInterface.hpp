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
  explicit PublisherInterface(PublisherType type);

  PublisherInterface(const PublisherInterface& publisher) = default;

  virtual ~PublisherInterface() = default;

  template<typename PubT, typename MsgT>
  std::shared_ptr<PublisherHandler<PubT, MsgT>> get_publisher(bool validate_pointer = true);

  void publish();

  void set_message_pair(const std::shared_ptr<MessagePairInterface>& message_pair);

  PublisherType get_type() const;

private:
  template<typename MsgT>
  void publish(const MsgT& message);

  PublisherType type_;
  std::shared_ptr<MessagePairInterface> message_pair_;
};

}// namespace modulo_new_core::communication
