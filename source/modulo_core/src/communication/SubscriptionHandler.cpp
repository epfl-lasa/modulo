#include "modulo_core/communication/SubscriptionHandler.hpp"

#include <utility>

namespace modulo_core::communication {

template<>
SubscriptionHandler<std_msgs::msg::Bool>::SubscriptionHandler(std::shared_ptr<MessagePairInterface> message_pair) :
    SubscriptionInterface(std::move(message_pair)), clock_(std::make_shared<rclcpp::Clock>()) {}

template<>
SubscriptionHandler<std_msgs::msg::Float64>::SubscriptionHandler(std::shared_ptr<MessagePairInterface> message_pair) :
    SubscriptionInterface(std::move(message_pair)), clock_(std::make_shared<rclcpp::Clock>()) {}
template<>
SubscriptionHandler<std_msgs::msg::Float64MultiArray>::SubscriptionHandler(
    std::shared_ptr<MessagePairInterface> message_pair
) :
    SubscriptionInterface(std::move(message_pair)), clock_(std::make_shared<rclcpp::Clock>()) {}

template<>
SubscriptionHandler<std_msgs::msg::Int32>::SubscriptionHandler(std::shared_ptr<MessagePairInterface> message_pair) :
    SubscriptionInterface(std::move(message_pair)), clock_(std::make_shared<rclcpp::Clock>()) {}

template<>
SubscriptionHandler<std_msgs::msg::String>::SubscriptionHandler(std::shared_ptr<MessagePairInterface> message_pair) :
    SubscriptionInterface(std::move(message_pair)), clock_(std::make_shared<rclcpp::Clock>()) {}

template<>
SubscriptionHandler<EncodedState>::SubscriptionHandler(std::shared_ptr<MessagePairInterface> message_pair) :
    SubscriptionInterface(std::move(message_pair)), clock_(std::make_shared<rclcpp::Clock>()) {}

template<>
std::function<void(const std::shared_ptr<std_msgs::msg::Bool>)>
SubscriptionHandler<std_msgs::msg::Bool>::get_callback() {
  return [this](const std::shared_ptr<std_msgs::msg::Bool> message) {
    try {
      this->get_message_pair()->template read<std_msgs::msg::Bool, bool>(*message);
    } catch (const exceptions::CoreException& ex) {
      RCLCPP_WARN_STREAM_THROTTLE(rclcpp::get_logger("SubscriptionHandler"), *this->clock_, 1000,
                                  "Exception in subscription callback: " << ex.what());
    }
  };
}

template<>
std::function<void(const std::shared_ptr<std_msgs::msg::Float64>)>
SubscriptionHandler<std_msgs::msg::Float64>::get_callback() {
  return [this](const std::shared_ptr<std_msgs::msg::Float64> message) {
    try {
      this->get_message_pair()->template read<std_msgs::msg::Float64, double>(*message);
    } catch (const exceptions::CoreException& ex) {
      RCLCPP_WARN_STREAM_THROTTLE(rclcpp::get_logger("SubscriptionHandler"), *this->clock_, 1000,
                                  "Exception in subscription callback: " << ex.what());
    }
  };
}

template<>
std::function<void(const std::shared_ptr<std_msgs::msg::Float64MultiArray>)>
SubscriptionHandler<std_msgs::msg::Float64MultiArray>::get_callback() {
  return [this](const std::shared_ptr<std_msgs::msg::Float64MultiArray> message) {
    try {
      this->get_message_pair()->template read<std_msgs::msg::Float64MultiArray, std::vector<double>>(*message);
    } catch (const exceptions::CoreException& ex) {
      RCLCPP_WARN_STREAM_THROTTLE(rclcpp::get_logger("SubscriptionHandler"), *this->clock_, 1000,
                                  "Exception in subscription callback: " << ex.what());
    }
  };
}

template<>
std::function<void(const std::shared_ptr<std_msgs::msg::Int32>)>
SubscriptionHandler<std_msgs::msg::Int32>::get_callback() {
  return [this](const std::shared_ptr<std_msgs::msg::Int32> message) {
    try {
      this->get_message_pair()->template read<std_msgs::msg::Int32, int>(*message);
    } catch (const exceptions::CoreException& ex) {
      RCLCPP_WARN_STREAM_THROTTLE(rclcpp::get_logger("SubscriptionHandler"), *this->clock_, 1000,
                                  "Exception in subscription callback: " << ex.what());
    }
  };
}

template<>
std::function<void(const std::shared_ptr<std_msgs::msg::String>)>
SubscriptionHandler<std_msgs::msg::String>::get_callback() {
  return [this](const std::shared_ptr<std_msgs::msg::String> message) {
    try {
      this->get_message_pair()->template read<std_msgs::msg::String, std::string>(*message);
    } catch (const exceptions::CoreException& ex) {
      RCLCPP_WARN_STREAM_THROTTLE(rclcpp::get_logger("SubscriptionHandler"), *this->clock_, 1000,
                                  "Exception in subscription callback: " << ex.what());
    }
  };
}

template<>
std::function<void(const std::shared_ptr<EncodedState>)> SubscriptionHandler<EncodedState>::get_callback() {
  return [this](const std::shared_ptr<EncodedState> message) {
    try {
      this->get_message_pair()->template read<EncodedState, state_representation::State>(*message);
    } catch (const exceptions::CoreException& ex) {
      RCLCPP_WARN_STREAM_THROTTLE(rclcpp::get_logger("SubscriptionHandler"), *this->clock_, 1000,
                                  "Exception in subscription callback: " << ex.what());
    }
  };
}
}// namespace modulo_core::communication
