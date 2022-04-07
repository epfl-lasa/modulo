#pragma once

#include "modulo_new_core/communication/MessagePairInterface.hpp"
#include "modulo_new_core/translators/WriteStateConversion.hpp"

#include <rclcpp/clock.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

namespace modulo_new_core::communication {

template<typename MsgT, typename DataT>
class MessagePair : public MessagePairInterface {
private:
  std::shared_ptr<DataT> data_;
  std::shared_ptr<rclcpp::Clock> clock_;

public:
  MessagePair(MessageType type, std::shared_ptr<DataT> data, std::shared_ptr<rclcpp::Clock> clock);

  [[nodiscard]] MsgT write_message() const;

  [[nodiscard]] std::shared_ptr<DataT> get_data() const;
};

template<typename MsgT, typename DataT>
MessagePair<MsgT, DataT>::MessagePair(
    MessageType type, std::shared_ptr<DataT> data, std::shared_ptr<rclcpp::Clock> clock
) :
    MessagePairInterface(type), data_(std::move(data)), clock_(std::move(clock)) {}

template<typename MsgT, typename DataT>
MsgT MessagePair<MsgT, DataT>::write_message() const {
  auto msg = MsgT();
  translators::write_msg(msg, *this->data_, clock_->now());
  return msg;
}

template<typename MsgT, typename DataT>
std::shared_ptr<DataT> MessagePair<MsgT, DataT>::get_data() const {
  return this->data_;
}
}// namespace modulo_new_core::communication
