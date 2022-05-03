#pragma once

#include <rclcpp/subscription.hpp>
#include "modulo_new_core/communication/MessagePair.hpp"

namespace modulo_new_core::communication {

// forward declaration of derived Subscription class
template<typename MsgT>
class SubscriptionHandler;

class SubscriptionInterface : public std::enable_shared_from_this<SubscriptionInterface> {
public:
  explicit SubscriptionInterface(std::shared_ptr<MessagePairInterface> message_pair = nullptr);

  SubscriptionInterface(const SubscriptionInterface& subscription) = default;

  virtual ~SubscriptionInterface() = default;

  template<typename MsgT>
  std::shared_ptr<SubscriptionHandler<MsgT>> get_handler(bool validate_pointer = true);

  [[nodiscard]] std::shared_ptr<MessagePairInterface> get_message_pair() const;

  void set_message_pair(const std::shared_ptr<MessagePairInterface>& message_pair);

private:
  std::shared_ptr<MessagePairInterface> message_pair_;
};

template<typename MsgT>
inline std::shared_ptr<SubscriptionHandler<MsgT>> SubscriptionInterface::get_handler(bool validate_pointer) {
  std::shared_ptr<SubscriptionHandler<MsgT>> subscription_ptr;
  try {
    subscription_ptr = std::dynamic_pointer_cast<SubscriptionHandler<MsgT>>(this->shared_from_this());
  } catch (const std::exception& ex) {
    if (validate_pointer) {
      throw exceptions::InvalidPointerException("Subscription interface is not managed by a valid pointer");
    }
  }
  if (subscription_ptr == nullptr && validate_pointer) {
    throw exceptions::InvalidPointerCastException(
        "Unable to cast subscription interface to a subscription pointer of requested type"
    );
  }
  return subscription_ptr;
}

}// namespace modulo_new_core::communication
