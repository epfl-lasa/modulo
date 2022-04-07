#pragma once

#include <state_representation/parameters/ParameterMap.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/create_timer.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <modulo_new_core/translators/ReadStateConversion.hpp>
#include <modulo_new_core/translators/WriteStateConversion.hpp>
#include <exceptions/PredicateNotFoundException.hpp>

#include "modulo_components/utilities/utilities.h"
#include "modulo_components/utilities/predicate_type.h"

namespace modulo_components {

template<class NodeT, typename PubT>
class ComponentInterface : public state_representation::ParameterMap, public NodeT {

public:
  explicit ComponentInterface(const rclcpp::NodeOptions& node_options);

protected:
  void add_predicate(const std::string& name, const std::function<bool(void)>& predicate);

  void add_predicate(const std::string& name, bool predicate);

  void set_predicate(const std::string& name, const std::function<bool(void)>& predicate);

  void set_predicate(const std::string& name, bool predicate);

  void send_transform(const state_representation::CartesianPose& transform);

  [[nodiscard]] state_representation::CartesianPose
  lookup_transform(const std::string& frame_name, const std::string& reference_frame_name = "world") const;

private:
  [[nodiscard]] std::string generate_predicate_topic(const std::string& predicate_name) const;

  void add_predicate(const std::string& name, const utilities::PredicateType& predicate);

  void set_predicate(const std::string& name, const utilities::PredicateType& predicate);

  void step();

  std::map<std::string, utilities::PredicateType> predicates_;
  std::map<std::string, std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>>> predicate_publishers_;

  std::shared_ptr<rclcpp::TimerBase> step_timer_;
  rclcpp::Parameter period_;
  rclcpp::Parameter has_tf_listener_;
  rclcpp::Parameter has_tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

template<class NodeT, class PubT>
ComponentInterface<NodeT, PubT>::ComponentInterface(const rclcpp::NodeOptions& options) :
    rclcpp::Node(utilities::parse_node_name(options, "ComponentInterface")) {
  this->declare_parameter("period", 1.0);
  this->declare_parameter("has_tf_listener", false);
  this->declare_parameter("has_tf_broadcaster", false);

  // here I need to do typename NodeT:: because ParameterMap also has get_parameter
  period_ = typename NodeT::get_parameter("has_tf_listener");
  has_tf_listener_ = typename NodeT::get_parameter("has_tf_listener");
  has_tf_broadcaster_ = typename NodeT::get_parameter("has_tf_broadcaster");
  if (has_tf_listener_.as_bool()) {
    this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);
  }
  if (has_tf_broadcaster_.as_bool()) {
    this->tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  }

  this->step_timer_ = this->create_wall_timer(
      std::chrono::nanoseconds(static_cast<int64_t>(this->period_.as_double() * 1e9)), [this] { this->step(); }
  );
}

template<class NodeT, typename PubT>
void ComponentInterface<NodeT, PubT>::step() {
  for (const auto& predicate: this->predicates_) {
    std_msgs::msg::Bool msg;
    switch (predicate.second.index()) {
      case 0:
        msg.data = std::get<0>(predicate.second);
      case 1:
        msg.data = std::get<1>(predicate.second)();
    }
    auto predicate_iterator = this->predicate_publishers_.find(predicate.first);
    if (predicate_iterator == this->predicate_publishers_.end()) {
      // TODO throw here
      RCLCPP_FATAL(this->get_logger(), "no publisher for predicate found");
      return;
    }
    predicate_publishers_.at(predicate.first)->publish(msg);
  }
}

template<class NodeT, class PubT>
std::string ComponentInterface<NodeT, PubT>::generate_predicate_topic(const std::string& predicate_name) const {
  return "/predicates/" + std::string(this->get_name()) + "/" + predicate_name;
}

template<class NodeT, typename PubT>
void
ComponentInterface<NodeT, PubT>::add_predicate(const std::string& name, const utilities::PredicateType& predicate) {
  if (this->predicates_.find(name) != this->predicates_.end()) {
    RCLCPP_INFO(this->get_logger(), "Predicate already exists, overwriting");
    this->predicates_.at(name) = predicate;
  } else {
    this->predicates_.insert(std::make_pair(name, predicate));
    // TODO add publisher with message interface
    this->predicate_publishers_.insert(
        std::make_pair(
            name, this->template create_publisher<std_msgs::msg::Bool>(this->generate_predicate_topic(name), 10)));
  }
}

template<class NodeT, typename PubT>
void ComponentInterface<NodeT, PubT>::add_predicate(const std::string& name, bool predicate) {
  this->add_predicate(name, utilities::PredicateType(predicate));
}

template<class NodeT, typename PubT>
void ComponentInterface<NodeT, PubT>::add_predicate(
    const std::string& name, const std::function<bool(void)>& predicate
) {
  this->add_predicate(name, utilities::PredicateType(predicate));
}

template<class NodeT, typename PubT>
void ComponentInterface<NodeT, PubT>::set_predicate(
    const std::string& name, const utilities::PredicateType& predicate
) {
  auto predicate_iterator = this->predicates_.find(name);
  if (predicate_iterator == this->predicates_.end()) {
    throw exceptions::PredicateNotFoundException(name);
  }
  this->predicates_.at(name) = predicate;
}

template<class NodeT, typename PubT>
void ComponentInterface<NodeT, PubT>::set_predicate(const std::string& name, bool predicate) {
  this->set_predicate(name, utilities::PredicateType(predicate));
}

template<class NodeT, typename PubT>
void ComponentInterface<NodeT, PubT>::set_predicate(
    const std::string& name, const std::function<bool(void)>& predicate
) {
  this->set_predicate(name, utilities::PredicateType(predicate));
}

template<class NodeT, typename PubT>
void ComponentInterface<NodeT, PubT>::send_transform(const state_representation::CartesianPose& transform) {
  // TODO: throw here?
  if (this->tf_broadcaster_ == nullptr) {
    RCLCPP_FATAL(this->get_logger(), "No tf broadcaster");
  }
  geometry_msgs::msg::TransformStamped tf_message;
  modulo_new_core::translators::write_msg(tf_message, transform, this->get_clock()->now());
  this->tf_broadcaster_->sendTransform(tf_message);
}

template<class NodeT, typename PubT>
state_representation::CartesianPose ComponentInterface<NodeT, PubT>::lookup_transform(
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
  modulo_new_core::translators::read_msg(result, transform);
  return result;
}

}// namespace modulo_components