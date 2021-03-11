#include "modulo_core/Communication/MessagePassing/TransformBroadcasterHandler.hpp"

namespace modulo::core::communication {
TransformBroadcasterHandler::TransformBroadcasterHandler(const std::shared_ptr<StateRepresentation::CartesianState>& recipient,
                                                         const std::shared_ptr<rclcpp::Clock>& clock,
                                                         const std::shared_ptr<std::mutex>& mutex) : PublisherHandler<StateRepresentation::CartesianState, tf2_msgs::msg::TFMessage>(recipient,
                                                                                                                                                                                     clock,
                                                                                                                                                                                     mutex) {}

TransformBroadcasterHandler::TransformBroadcasterHandler(const std::shared_ptr<rclcpp::Clock>& clock,
                                                         const std::shared_ptr<std::mutex>& mutex) : PublisherHandler<StateRepresentation::CartesianState, tf2_msgs::msg::TFMessage>(std::make_shared<StateRepresentation::CartesianState>(),
                                                                                                                                                                                     clock,
                                                                                                                                                                                     mutex) {}
}// namespace modulo::core::communication
