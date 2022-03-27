#include "modulo_core/communication/message_passing/TransformListenerHandler.hpp"

namespace modulo::core::communication {
const state_representation::CartesianPose TransformListenerHandler::lookup_transform(const std::string& frame_name,
                                                                                     const std::string& reference_frame) const {
  geometry_msgs::msg::TransformStamped transformStamped;
  state_representation::CartesianPose result(frame_name, reference_frame);
  transformStamped = this->buffer_.lookupTransform(reference_frame,
                                                   frame_name,
                                                   tf2::TimePoint(std::chrono::milliseconds(0)),
                                                   tf2::Duration(this->get_timeout()));
  modulo_new_core::translators::read_msg(result, transformStamped);
  return result;
}
}// namespace modulo::core::communication
