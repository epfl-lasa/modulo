#pragma once

#include "modulo_core/Communication/MessagePassing/PublisherHandler.hpp"
#include "state_representation/Space/Cartesian/CartesianState.hpp"
#include <tf2_msgs/msg/tf_message.hpp>

namespace modulo::core::communication {
/**
 * @class TransformBroadcasterHandler
 * @brief Class to define a transform broadcaster
 */
class TransformBroadcasterHandler : public PublisherHandler<StateRepresentation::CartesianState, tf2_msgs::msg::TFMessage> {
public:
  /**
   * @brief Constructor for and asychronous TransformBroadcaster handler
   * @param  recipient the associated recipient to publish
   * @param  clock     reference to the Cell clock
   * @param  mutex     reference to the Cell mutex
   */
  explicit TransformBroadcasterHandler(const std::shared_ptr<StateRepresentation::CartesianState>& recipient,
                                       const std::shared_ptr<rclcpp::Clock>& clock,
                                       const std::shared_ptr<std::mutex>& mutex);

  /**
   * @brief Constructor for TransformBroadcaster handler without an associated recipient
   * @param  clock     reference to the Cell clock
   * @param  mutex     reference to the Cell mutex
   */
  explicit TransformBroadcasterHandler(const std::shared_ptr<rclcpp::Clock>& clock, const std::shared_ptr<std::mutex>& mutex);

  /**
   * @brief Function to send a transform over the network
   * @param transform the transformation to send
   */
  void send_transform(const StateRepresentation::CartesianState& transform);
};

inline void TransformBroadcasterHandler::send_transform(const StateRepresentation::CartesianState& transform) {
  this->publish(transform);
}
}// namespace modulo::core::communication
