#include "modulo_new_core/communication/SubscriptionHandler.hpp"

#include <utility>

namespace modulo_new_core::communication {

template<>
SubscriptionHandler<std_msgs::msg::Bool>::SubscriptionHandler(std::shared_ptr<MessagePairInterface> message_pair) :
    SubscriptionInterface(std::move(message_pair)) {}

template<>
SubscriptionHandler<std_msgs::msg::Float64>::SubscriptionHandler(std::shared_ptr<MessagePairInterface> message_pair) :
    SubscriptionInterface(std::move(message_pair)) {}
template<>
SubscriptionHandler<std_msgs::msg::Float64MultiArray>::SubscriptionHandler(
    std::shared_ptr<MessagePairInterface> message_pair
) :
    SubscriptionInterface(std::move(message_pair)) {}

template<>
SubscriptionHandler<std_msgs::msg::Int32>::SubscriptionHandler(std::shared_ptr<MessagePairInterface> message_pair) :
    SubscriptionInterface(std::move(message_pair)) {}

template<>
SubscriptionHandler<std_msgs::msg::String>::SubscriptionHandler(std::shared_ptr<MessagePairInterface> message_pair) :
    SubscriptionInterface(std::move(message_pair)) {}

template<>
SubscriptionHandler<EncodedState>::SubscriptionHandler(std::shared_ptr<MessagePairInterface> message_pair) :
    SubscriptionInterface(std::move(message_pair)) {}

template<>
std::function<void(const std::shared_ptr<std_msgs::msg::Bool>)>
SubscriptionHandler<std_msgs::msg::Bool>::get_callback() {
  return [this](const std::shared_ptr<std_msgs::msg::Bool> message) {
    this->get_message_pair()->template read<std_msgs::msg::Bool, bool>(*message);
  };
}

template<>
std::function<void(const std::shared_ptr<std_msgs::msg::Float64>)>
SubscriptionHandler<std_msgs::msg::Float64>::get_callback() {
  return [this](const std::shared_ptr<std_msgs::msg::Float64> message) {
    this->get_message_pair()->template read<std_msgs::msg::Float64, double>(*message);
  };
}

template<>
std::function<void(const std::shared_ptr<std_msgs::msg::Float64MultiArray>)>
SubscriptionHandler<std_msgs::msg::Float64MultiArray>::get_callback() {
  return [this](const std::shared_ptr<std_msgs::msg::Float64MultiArray> message) {
    this->get_message_pair()->template read<std_msgs::msg::Float64MultiArray, std::vector<double>>(*message);
  };
}

template<>
std::function<void(const std::shared_ptr<std_msgs::msg::Int32>)>
SubscriptionHandler<std_msgs::msg::Int32>::get_callback() {
  return [this](const std::shared_ptr<std_msgs::msg::Int32> message) {
    this->get_message_pair()->template read<std_msgs::msg::Int32, int>(*message);
  };
}

template<>
std::function<void(const std::shared_ptr<std_msgs::msg::String>)>
SubscriptionHandler<std_msgs::msg::String>::get_callback() {
  return [this](const std::shared_ptr<std_msgs::msg::String> message) {
    this->get_message_pair()->template read<std_msgs::msg::String, std::string>(*message);
  };
}

template<>
std::function<void(const std::shared_ptr<EncodedState>)> SubscriptionHandler<EncodedState>::get_callback() {
  return [this](const std::shared_ptr<EncodedState> message) {
    this->get_message_pair()->template read<EncodedState, state_representation::State>(*message);
  };
}

}// namespace modulo_new_core::communication