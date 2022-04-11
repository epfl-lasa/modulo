#pragma once

#include "modulo_new_core/communication/MessagePairInterface.hpp"
#include "modulo_new_core/exceptions/NullPointerException.hpp"
#include "modulo_new_core/translators/WriteStateConversion.hpp"

#include <rclcpp/clock.hpp>

namespace modulo_new_core::communication {

template<typename MsgT, typename DataT>
class MessagePair : public MessagePairInterface {
public:
  MessagePair(std::shared_ptr<DataT> data, std::shared_ptr<rclcpp::Clock> clock);

  [[nodiscard]] MsgT write_message() const;

  [[nodiscard]] std::shared_ptr<DataT> get_data() const;

private:
  std::shared_ptr<DataT> data_;
  std::shared_ptr<rclcpp::Clock> clock_;
};

template<typename MsgT, typename DataT>
inline MsgT MessagePair<MsgT, DataT>::write_message() const {
  if (this->data_ == nullptr) {
    throw exceptions::NullPointerException("The message pair data is not set, nothing to write");
  }
  auto msg = MsgT();
  translators::write_msg(msg, *this->data_, clock_->now());
  return msg;
}

template<>
inline EncodedState MessagePair<EncodedState, state_representation::State>::write_message() const {
  auto msg = EncodedState();
  // TODO write the translators
//  translators::write_msg(msg, this->data_, clock_->now());
  return msg;
}

template<typename MsgT, typename DataT>
inline std::shared_ptr<DataT> MessagePair<MsgT, DataT>::get_data() const {
  return this->data_;
}
}// namespace modulo_new_core::communication
