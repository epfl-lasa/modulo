#pragma once

#include "modulo_new_core/communication/PublisherInterface.hpp"
#include "modulo_new_core/exceptions/NotLifecyclePublisherException.hpp"

namespace modulo_new_core::communication {

template<typename PubT, typename MsgT>
class PublisherHandler : public PublisherInterface {
public:
  PublisherHandler(PublisherType type, std::shared_ptr<PubT> publisher);

  void on_activate();

  void on_deactivate();

  void publish(const MsgT& message) const;

  std::shared_ptr<PublisherInterface>
  create_publisher_interface(const std::shared_ptr<MessagePairInterface>& message_pair);

private:
  std::shared_ptr<PubT> publisher_;
};

template<typename PubT, typename MsgT>
PublisherHandler<PubT, MsgT>::PublisherHandler(PublisherType type, std::shared_ptr<PubT> publisher) :
    PublisherInterface(type), publisher_(std::move(publisher)) {}

template<typename PubT, typename MsgT>
void PublisherHandler<PubT, MsgT>::on_activate() {
  if (this->get_type() != PublisherType::LIFECYCLE_PUBLISHER) {
    throw exceptions::NotLifecyclePublisherException("Only LifecyclePublishers can be activated");
  }
  this->publisher_->on_activate();
}

template<typename PubT, typename MsgT>
void PublisherHandler<PubT, MsgT>::on_deactivate() {
  if (this->get_type() != PublisherType::LIFECYCLE_PUBLISHER) {
    throw exceptions::NotLifecyclePublisherException("Only LifecyclePublishers can be deactivated");
  }
  this->publisher_->on_deactivate();
}

template<typename PubT, typename MsgT>
void PublisherHandler<PubT, MsgT>::publish(const MsgT& message) const {
  if (this->publisher_ == nullptr) {
    throw exceptions::NullPointerException("Publisher not set");
  }
  this->publisher_->publish(message);
}

template<typename PubT, typename MsgT>
std::shared_ptr<PublisherInterface> PublisherHandler<PubT, MsgT>::create_publisher_interface(
    const std::shared_ptr<MessagePairInterface>& message_pair
) {
  std::shared_ptr<PublisherInterface> publisher_interface(this->shared_from_this());
  publisher_interface->set_message_pair(message_pair);
  return publisher_interface;
}

}// namespace modulo_new_core::communication
