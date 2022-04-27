#pragma once

#include "modulo_new_core/communication/MessagePairInterface.hpp"
#include "modulo_new_core/exceptions/NullPointerException.hpp"
#include "modulo_new_core/translators/message_readers.hpp"
#include "modulo_new_core/translators/message_writers.hpp"

#include <rclcpp/clock.hpp>

namespace modulo_new_core::communication {

template<typename MsgT, typename DataT>
class MessagePair : public MessagePairInterface {
public:
  MessagePair(std::shared_ptr<DataT> data, std::shared_ptr<rclcpp::Clock> clock);

  [[nodiscard]] MsgT write_message() const;

  void read_message(const MsgT& message);

  [[nodiscard]] std::shared_ptr<DataT> get_data() const;

  void set_data(const std::shared_ptr<DataT>& data);

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
  if (this->data_ == nullptr) {
    throw exceptions::NullPointerException("The message pair data is not set, nothing to write");
  }
  auto msg = EncodedState();
  translators::write_msg(msg, this->data_, clock_->now());
  return msg;
}

template<typename MsgT, typename DataT>
inline void MessagePair<MsgT, DataT>::read_message(const MsgT& message) {
  if (this->data_ == nullptr) {
    throw exceptions::NullPointerException("The message pair data is not set, nothing to read");
  }
  translators::read_msg(*this->data_, message);
}

template<>
inline void MessagePair<EncodedState, state_representation::State>::read_message(const EncodedState& message) {
  if (this->data_ == nullptr) {
    throw exceptions::NullPointerException("The message pair data is not set, nothing to read");
  }
  translators::read_msg(this->data_, message);
}

template<typename MsgT, typename DataT>
inline std::shared_ptr<DataT> MessagePair<MsgT, DataT>::get_data() const {
  return this->data_;
}

template<typename MsgT, typename DataT>
inline void MessagePair<MsgT, DataT>::set_data(const std::shared_ptr<DataT>& data) {
  if (data == nullptr) {
    throw exceptions::NullPointerException("Provide a valid pointer");
  }
  this->data_ = data;
}

template<typename DataT>
std::shared_ptr<MessagePairInterface>
make_shared_message_pair(const std::shared_ptr<DataT>& data, const std::shared_ptr<rclcpp::Clock>& clock);

}// namespace modulo_new_core::communication
