#pragma once

#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "modulo_core/communication/message_passing/MessagePassingHandler.hpp"

namespace modulo::core::communication {
/**
 * @class TransformListenerHandler
 * @brief Class to define a transform listener
 */
class TransformListenerHandler : public MessagePassingHandler {
private:
  tf2_ros::Buffer buffer_;                                 ///< tf2 ROS buffer to read transformation from
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;///< reference to the ROS2 transform listener

public:
  /**
   * @brief Constructor for an asychronous TransformListener
   * @param  recipient the associated recipient to store received transforms
   * @param  timeout   period before timeout
   * @param  clock     reference to the Cell clock
   */
  template <typename DurationT>
  explicit TransformListenerHandler(const std::shared_ptr<state_representation::CartesianPose>& recipient,
                                    const std::chrono::duration<int64_t, DurationT>& timeout,
                                    const std::shared_ptr<rclcpp::Clock>& clock);

  /**
   * @brief Constructor for a TransformListener without a recipient
   * @param  timeout   period before timeout
   * @param  clock     reference to the Cell clock
   */
  template <typename DurationT>
  explicit TransformListenerHandler(const std::chrono::duration<int64_t, DurationT>& timeout,
                                    const std::shared_ptr<rclcpp::Clock>& clock);

  /**
   * @brief Function to look up a transform over the network
   * @param  frame_name      name of the frame associated to the transform
   * @param  reference_frame name of its desired reference frame
   * @return                 the transform as a CartesianPose
   */
  const state_representation::CartesianPose lookup_transform(const std::string& frame_name,
                                                             const std::string& reference_frame) const;
};

template <typename DurationT>
TransformListenerHandler::TransformListenerHandler(const std::shared_ptr<state_representation::CartesianPose>& recipient,
                                                   const std::chrono::duration<int64_t, DurationT>& timeout,
                                                   const std::shared_ptr<rclcpp::Clock>& clock) : MessagePassingHandler(CommunicationType::TRANSFORMLISTENER,
                                                                                                                        recipient,
                                                                                                                        timeout),
                                                                                                  buffer_(clock) {
  this->tf_listener_ = std::make_unique<tf2_ros::TransformListener>(buffer_);
}

template <typename DurationT>
TransformListenerHandler::TransformListenerHandler(const std::chrono::duration<int64_t, DurationT>& timeout,
                                                   const std::shared_ptr<rclcpp::Clock>& clock) : MessagePassingHandler(CommunicationType::TRANSFORMLISTENER,
                                                                                                                        std::make_shared<state_representation::CartesianPose>(),
                                                                                                                        timeout),
                                                                                                  buffer_(clock) {
  this->tf_listener_ = std::make_unique<tf2_ros::TransformListener>(buffer_);
}
}// namespace modulo::core::communication
