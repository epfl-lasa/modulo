#pragma once

#include "modulo_new_core/communication/PublisherInterface.hpp"

namespace modulo_new_core::communication {

template<typename PubT, typename MsgT>
class PublisherHandler : public PublisherInterface {
public:
  PublisherHandler(PublisherType type, std::shared_ptr<PubT> publisher);

  void publish(const MsgT& message) const;

private:
  std::shared_ptr<PubT> publisher_;
};

template<typename PubT, typename MsgT>
PublisherHandler<PubT, MsgT>::PublisherHandler(PublisherType type, std::shared_ptr<PubT> publisher) :
    PublisherInterface(type), publisher_(std::move(publisher)) {}

template<typename PubT, typename MsgT>
void PublisherHandler<PubT, MsgT>::publish(const MsgT& message) const {
  this->publisher_->publish(message);
}

}// namespace modulo_new_core::communication
