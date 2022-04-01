#pragma once

#include "modulo_new_core/communication/MessagePairInterface.hpp"

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/string.hpp>

namespace modulo_new_core::communication {

template<typename MsgT, typename DataT>
class MessagePair : public MessagePairInterface {
private:
  std::shared_ptr<DataT> data_;

public:
  explicit MessagePair(std::shared_ptr<DataT> data);

  [[nodiscard]] MsgT get_message() const;

  [[nodiscard]] std::shared_ptr<DataT> get_data() const;
};

template<typename MsgT, typename DataT>
MsgT MessagePair<MsgT, DataT>::get_message() const {
  auto msg = MsgT();
  msg.data = *this->data_;
  return msg;
}

template<typename MsgT, typename DataT>
std::shared_ptr<DataT> MessagePair<MsgT, DataT>::get_data() const {
  return this->data_;
}
}// namespace modulo_new_core::communication
