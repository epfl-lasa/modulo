#pragma once

#include "modulo_core/Communication/MessagePassing/PublisherHandler.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"
#include <tf2_msgs/msg/tf_message.hpp>

namespace modulo::core::communication {
/**
 * @class TransformBroadcasterHandler
 * @brief Class to define a transform broadcaster
 */
class TransformBroadcasterHandler : public PublisherHandler<state_representation::CartesianState, tf2_msgs::msg::TFMessage> {
public:
  /**
   * @brief Constructor for and asychronous TransformBroadcaster handler
   * @param  recipient the associated recipient to publish
   * @param  clock     reference to the Cell clock
   */
  explicit TransformBroadcasterHandler(const std::shared_ptr<state_representation::CartesianState>& recipient,
                                       const std::shared_ptr<rclcpp::Clock>& clock);

  /**
   * @brief Constructor for TransformBroadcaster handler without an associated recipient
   * @param  clock     reference to the Cell clock
   */
  explicit TransformBroadcasterHandler(const std::shared_ptr<rclcpp::Clock>& clock);

  /**
   * @brief Function to send a transform over the network
   * @param transform the transformation to send
   */
  void send_transform(const state_representation::CartesianState& transform);
};

inline void TransformBroadcasterHandler::send_transform(const state_representation::CartesianState& transform) {
  this->publish(transform);
}
}// namespace modulo::core::communication
