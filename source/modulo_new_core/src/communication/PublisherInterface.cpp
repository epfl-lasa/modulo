#include "modulo_new_core/communication/PublisherInterface.hpp"

#include <rclcpp/publisher.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include "modulo_new_core/communication/PublisherHandler.hpp"
#include "modulo_new_core/exceptions/NullPointerException.hpp"

namespace modulo_new_core::communication {

PublisherInterface::PublisherInterface(PublisherType type) : type_(type) {}

void PublisherInterface::publish() {
  if (this->message_pair_ == nullptr) {
    throw exceptions::NullPointerException("Message pair is not set, nothing to publish");
  }
  switch (this->message_pair_->get_type()) {
    case MessageType::BOOL:
      this->publish(this->message_pair_->write<std_msgs::msg::Bool, bool>());
      break;
    case MessageType::FLOAT64:
      this->publish(this->message_pair_->write<std_msgs::msg::Float64, double>());
      break;
    case MessageType::FLOAT64_MULTI_ARRAY:
      this->publish(this->message_pair_->write<std_msgs::msg::Float64MultiArray, std::vector<double>>());
      break;
    case MessageType::INT32:
      this->publish(this->message_pair_->write<std_msgs::msg::Int32, int>());
      break;
    case MessageType::STRING:
      this->publish(this->message_pair_->write<std_msgs::msg::String, std::string>());
      break;
    case MessageType::ENCODED_STATE:
      this->publish(this->message_pair_->write<EncodedState, state_representation::State>());
      break;
  }
}

template<typename MsgT>
void PublisherInterface::publish(const MsgT& message) {
  switch (this->get_type()) {
    case PublisherType::PUBLISHER:
      this->template get_publisher<rclcpp::Publisher<MsgT>, MsgT>()->publish(message);
      break;
    case PublisherType::LIFECYCLE_PUBLISHER:
      this->template get_publisher<rclcpp_lifecycle::LifecyclePublisher<MsgT>, MsgT>()->publish(message);
      break;
  }
}

void PublisherInterface::set_message_pair(const std::shared_ptr<MessagePairInterface>& message_pair) {
  if (message_pair == nullptr) {
    throw exceptions::NullPointerException("Provide a valid pointer");
  }
  this->message_pair_ = message_pair;
}

PublisherType PublisherInterface::get_type() const {
  return this->type_;
}

}// namespace modulo_new_core::communication