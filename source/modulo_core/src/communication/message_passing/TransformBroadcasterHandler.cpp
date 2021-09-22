#include "modulo_core/communication/message_passing/TransformBroadcasterHandler.hpp"

namespace modulo::core::communication {
TransformBroadcasterHandler::TransformBroadcasterHandler(const std::shared_ptr<state_representation::CartesianState>& recipient,
                                                         const std::shared_ptr<rclcpp::Clock>& clock) : PublisherHandler<state_representation::CartesianState, tf2_msgs::msg::TFMessage>(recipient,
                                                                                                                                                                                         clock) {}

TransformBroadcasterHandler::TransformBroadcasterHandler(const std::shared_ptr<rclcpp::Clock>& clock) : PublisherHandler<state_representation::CartesianState, tf2_msgs::msg::TFMessage>(std::make_shared<state_representation::CartesianState>(),
                                                                                                                                                                                         clock) {}
}// namespace modulo::core::communication
