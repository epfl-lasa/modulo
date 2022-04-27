#include "modulo_new_core/communication/MessagePair.hpp"

namespace modulo_new_core::communication {

template<>
MessagePair<std_msgs::msg::Bool, bool>::MessagePair(
    std::shared_ptr<bool> data, std::shared_ptr<rclcpp::Clock> clock
) :
    MessagePairInterface(MessageType::BOOL), data_(std::move(data)), clock_(std::move(clock)) {}

template<>
MessagePair<std_msgs::msg::Float64, double>::MessagePair(
    std::shared_ptr<double> data, std::shared_ptr<rclcpp::Clock> clock
) :
    MessagePairInterface(MessageType::FLOAT64), data_(std::move(data)), clock_(std::move(clock)) {}

template<>
MessagePair<std_msgs::msg::Float64MultiArray, std::vector<double>>::MessagePair(
    std::shared_ptr<std::vector<double>> data, std::shared_ptr<rclcpp::Clock> clock
) :
    MessagePairInterface(MessageType::FLOAT64_MULTI_ARRAY), data_(std::move(data)), clock_(std::move(clock)) {}

template<>
MessagePair<std_msgs::msg::Int32, int>::MessagePair(
    std::shared_ptr<int> data, std::shared_ptr<rclcpp::Clock> clock
) :
    MessagePairInterface(MessageType::INT32), data_(std::move(data)), clock_(std::move(clock)) {}

template<>
MessagePair<std_msgs::msg::String, std::string>::MessagePair(
    std::shared_ptr<std::string> data, std::shared_ptr<rclcpp::Clock> clock
) :
    MessagePairInterface(MessageType::STRING), data_(std::move(data)), clock_(std::move(clock)) {}

template<>
MessagePair<EncodedState, state_representation::State>::MessagePair(
    std::shared_ptr<state_representation::State> data, std::shared_ptr<rclcpp::Clock> clock
) :
    MessagePairInterface(MessageType::ENCODED_STATE), data_(std::move(data)), clock_(std::move(clock)) {}

template<>
std::shared_ptr<MessagePairInterface>
make_shared_message_pair(const std::shared_ptr<bool>& data, const std::shared_ptr<rclcpp::Clock>& clock) {
  return std::make_shared<MessagePair<std_msgs::msg::Bool, bool>>(data, clock);
}

template<>
std::shared_ptr<MessagePairInterface>
make_shared_message_pair(const std::shared_ptr<double>& data, const std::shared_ptr<rclcpp::Clock>& clock) {
  return std::make_shared<MessagePair<std_msgs::msg::Float64, double>>(data, clock);
}

template<>
std::shared_ptr<MessagePairInterface> make_shared_message_pair(
    const std::shared_ptr<std::vector<double>>& data, const std::shared_ptr<rclcpp::Clock>& clock
) {
  return std::make_shared<MessagePair<std_msgs::msg::Float64MultiArray, std::vector<double>>>(data, clock);
}

template<>
std::shared_ptr<MessagePairInterface>
make_shared_message_pair(const std::shared_ptr<int>& data, const std::shared_ptr<rclcpp::Clock>& clock) {
  return std::make_shared<MessagePair<std_msgs::msg::Int32, int>>(data, clock);
}

template<>
std::shared_ptr<MessagePairInterface>
make_shared_message_pair(const std::shared_ptr<std::string>& data, const std::shared_ptr<rclcpp::Clock>& clock) {
  return std::make_shared<MessagePair<std_msgs::msg::String, std::string>>(data, clock);
}

template<>
std::shared_ptr<MessagePairInterface> make_shared_message_pair(
    const std::shared_ptr<state_representation::State>& data, const std::shared_ptr<rclcpp::Clock>& clock
) {
  return std::make_shared<MessagePair<EncodedState, state_representation::State>>(data, clock);
}

}// namespace modulo_new_core::communication