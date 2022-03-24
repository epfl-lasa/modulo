#pragma once

#include <state_representation/parameters/ParameterMap.hpp>
#include <state_representation/parameters/Predicate.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/subscription.hpp>
#include <modulo_core/communication/EncodedState.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <modulo_core/communication/message_passing/ReadStateConversion.hpp>
#include <modulo_core/communication/message_passing/WriteStateConversion.hpp>

#include "modulo_components/utilities/utilities.h"
#include "modulo_components/utilities/predicate_type.h"

namespace modulo_components {

template<class T, typename PubT>
class ComponentInterface : public state_representation::ParameterMap, public T {
public:
  explicit ComponentInterface(const rclcpp::NodeOptions& node_options);

protected:
  void add_predicate(const std::string& name, bool value, const std::function<bool(void)>& function);

  void send_transform(const state_representation::CartesianState& transform);

  [[nodiscard]] state_representation::CartesianState
  lookup_transform(const std::string& frame_name, const std::string& reference_frame_name = "world") const;

private:
  std::map<std::string, PubT> publishers_;
  std::map<std::string, std::shared_ptr<state_representation::Predicate>> predicates_;
  std::map<std::string, rclcpp::Publisher<bool>> predicate_publishers_;
  std::map<std::string, rclcpp::Subscription<modulo::core::EncodedState>> subscribers_;
  std::map<std::string, std::function<bool(void)>> predicate_functions_;

  rclcpp::Parameter period_;
  rclcpp::Parameter has_tf_listener_;
  rclcpp::Parameter has_tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

template<class T, class PubT>
ComponentInterface<T, PubT>::ComponentInterface(const rclcpp::NodeOptions& options) :
    T(utilities::parse_node_name(options, "ComponentInterface")) {
  this->declare_parameter("period", 1.0);
  this->declare_parameter("has_tf_listener", false);
  this->declare_parameter("has_tf_broadcaster", false);

  period_ = T::get_parameter("has_tf_listener");
  has_tf_listener_ = T::get_parameter("has_tf_listener");
  has_tf_broadcaster_ = T::get_parameter("has_tf_broadcaster");
  if (has_tf_listener_.as_bool()) {
    this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);
  }
  if (has_tf_broadcaster_.as_bool()) {
    this->tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  }
}

template<class T, typename PubT>
void ComponentInterface<T, PubT>::add_predicate(
    const std::string& name, bool value, const std::function<bool()>& function
) {
  this->predicate_functions_.template insert_or_assign(name, function);
  if (this->predicates_.find(name) != this->predicates_.end()) {
    std::cerr << "predicate already exists, overwriting callback function" << std::endl;
    return;
  }
  this->predicates_.insert(std::make_pair(name, std::make_shared<state_representation::Predicate>(name, value)));
  // TODO add publisher
}

template<class T, typename PubT>
void ComponentInterface<T, PubT>::send_transform(const state_representation::CartesianState& transform) {
  // TODO: throw here?
  if (this->tf_broadcaster_ == nullptr) {
    RCLCPP_FATAL(this->get_logger(), "No tf broadcaster");
  }
  geometry_msgs::msg::TransformStamped tf_message;
  modulo::core::communication::state_conversion::write_msg(tf_message, transform, this->get_clock()->now());
  this->tf_broadcaster_->sendTransform(tf_message);
}

template<class T, typename PubT>
state_representation::CartesianState ComponentInterface<T, PubT>::lookup_transform(
    const std::string& frame_name, const std::string& reference_frame_name
) const {
  // TODO: throw here?
  if (this->tf_buffer_ == nullptr || this->tf_listener_ == nullptr) {
    RCLCPP_FATAL(this->get_logger(), "No tf buffer / listener");
  }
  geometry_msgs::msg::TransformStamped transform;
  state_representation::CartesianPose result(frame_name, reference_frame_name);
  // TODO: timeout
  transform = this->tf_buffer_->lookupTransform(
      reference_frame_name, frame_name, tf2::TimePoint(std::chrono::microseconds(0)),
      tf2::Duration(std::chrono::microseconds(10)));
  modulo::core::communication::state_conversion::read_msg(result, transform);
  return result;
}

}// namespace modulo_components