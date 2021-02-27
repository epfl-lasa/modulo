#pragma once

#include "modulo_core/Communication/MessagePassing/MessagePassingHandler.hpp"
#include <tf2_ros/transform_listener.h>

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
 * @param  mutex     reference to the Cell mutex
 */
  template <typename DurationT>
  explicit TransformListenerHandler(const std::shared_ptr<StateRepresentation::CartesianPose>& recipient,
                                    const std::chrono::duration<int64_t, DurationT>& timeout,
                                    const std::shared_ptr<rclcpp::Clock>& clock,
                                    const std::shared_ptr<std::mutex>& mutex);

  /**
 * @brief Constructor for a TransformListener without a recipient
 * @param  timeout   period before timeout
 * @param  clock     reference to the Cell clock
 * @param  mutex     reference to the Cell mutex
 */
  template <typename DurationT>
  explicit TransformListenerHandler(const std::chrono::duration<int64_t, DurationT>& timeout,
                                    const std::shared_ptr<rclcpp::Clock>& clock,
                                    const std::shared_ptr<std::mutex>& mutex);

  /**
 * @brief Function to look up a transform over the network
 * @param  frame_name      name of the frame associated to the transform
 * @param  reference_frame name of its desired reference frame
 * @return                 the transform as a CartesianPose
 */
  const StateRepresentation::CartesianPose lookup_transform(const std::string& frame_name,
                                                            const std::string& reference_frame) const;
};

template <typename DurationT>
TransformListenerHandler::TransformListenerHandler(const std::shared_ptr<StateRepresentation::CartesianPose>& recipient,
                                                   const std::chrono::duration<int64_t, DurationT>& timeout,
                                                   const std::shared_ptr<rclcpp::Clock>& clock,
                                                   const std::shared_ptr<std::mutex>& mutex) : MessagePassingHandler(CommunicationType::TRANSFORMLISTENER,
                                                                                                                     recipient,
                                                                                                                     timeout,
                                                                                                                     mutex),
                                                                                               buffer_(clock) {
  this->tf_listener_ = std::make_unique<tf2_ros::TransformListener>(buffer_);
}

template <typename DurationT>
TransformListenerHandler::TransformListenerHandler(const std::chrono::duration<int64_t, DurationT>& timeout,
                                                   const std::shared_ptr<rclcpp::Clock>& clock,
                                                   const std::shared_ptr<std::mutex>& mutex) : MessagePassingHandler(CommunicationType::TRANSFORMLISTENER,
                                                                                                                     std::make_shared<StateRepresentation::CartesianPose>(),
                                                                                                                     timeout,
                                                                                                                     mutex),
                                                                                               buffer_(clock) {
  this->tf_listener_ = std::make_unique<tf2_ros::TransformListener>(buffer_);
}
}// namespace modulo::core::communication
