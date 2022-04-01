#include "modulo_new_core/communication/MessagePair.hpp"

#include <utility>

namespace modulo_new_core::communication {

template<>
MessagePair<std_msgs::msg::Bool, bool>::MessagePair(std::shared_ptr<bool>  data) :
    MessagePairInterface(MessageType::BOOL), data_(std::move(data)) {}

template<>
MessagePair<std_msgs::msg::Float64, double>::MessagePair(std::shared_ptr<double>  data) :
    MessagePairInterface(MessageType::FLOAT64), data_(std::move(data)) {}

template<>
MessagePair<std_msgs::msg::Float64MultiArray, std::vector<double>>::MessagePair(std::shared_ptr<std::vector<double>>  data) :
    MessagePairInterface(MessageType::FLOAT64_MULTI_ARRAY), data_(std::move(data)) {}

template<>
MessagePair<std_msgs::msg::Int64, int>::MessagePair(std::shared_ptr<int>  data) :
    MessagePairInterface(MessageType::INT64), data_(std::move(data)) {}

template<>
MessagePair<std_msgs::msg::String, std::string>::MessagePair(std::shared_ptr<std::string>  data) :
    MessagePairInterface(MessageType::STRING), data_(std::move(data)) {}

}// namespace modulo_new_core::communication