#pragma once

#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/node_options.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <modulo_new_core/communication/PublisherType.hpp>
#include <modulo_new_core/translators/ReadStateConversion.hpp>
#include <modulo_new_core/translators/WriteStateConversion.hpp>

#include "modulo_components/exceptions/PredicateNotFoundException.hpp"
#include "modulo_components/utilities/utilities.h"
#include "modulo_components/utilities/predicate_variant.h"

namespace modulo_components {

template<class NodeT, typename>
class ComponentInterface : NodeT {
  friend class ComponentInterfaceTest;

public:
  /**
   * @brief Constructor from node options
   * @param node_options node options as used in ROS2 Node
   */
  explicit ComponentInterface(
      const rclcpp::NodeOptions& node_options, modulo_new_core::communication::PublisherType publisher_type
  );

protected:
  /**
   * @brief Add a predicate to the map of predicates
   * @param predicate_name the name of the associated predicate
   * @param predicate_value the boolean value of the predicate
   */
  void add_predicate(const std::string& predicate_name, bool predicate_value);

  /**
   * @brief Add a predicate to the map of predicates based on a function to periodically call
   * @param predicate_name the name of the associated predicate
   * @param predicate_function the function to call that returns the value of the predicate
   */
  void add_predicate(const std::string& predicate_name, const std::function<bool(void)>& predicate_function);

  /**
   * @brief Get the value of the predicate given as parameter
   * @param predicate_name the name of the predicate to retrieve from the
   * map of predicates
   * @return the value of the predicate as a boolean
   */
  [[nodiscard]] bool get_predicate(const std::string& predicate_name) const;

  /**
   * @brief Set the value of the predicate given as parameter
   * @param predicate_name the name of the predicate to retrieve from the
   * map of predicates
   * @param predicate_value the new value of the predicate
   */
  void set_predicate(const std::string& predicate_name, bool predicate_value);

  /**
   * @brief Set the value of the predicate given as parameter
   * @param predicate_name the name of the predicate to retrieve from the
   * map of predicates
   * @param predicate_function the function to periodically call that returns the value of the predicate
   */
  void set_predicate(const std::string& predicate_name, const std::function<bool(void)>& predicate_function);

  void send_transform(const state_representation::CartesianPose& transform);

  [[nodiscard]] state_representation::CartesianPose
  lookup_transform(const std::string& frame_name, const std::string& reference_frame_name = "world") const;

private:
  [[nodiscard]] std::string generate_predicate_topic(const std::string& predicate_name) const;

  void add_variant_predicate(const std::string& name, const utilities::PredicateVariant& predicate);

  void set_variant_predicate(const std::string& name, const utilities::PredicateVariant& predicate);

  void step();

  modulo_new_core::communication::PublisherType publisher_type_;

  std::map<std::string, utilities::PredicateVariant> predicates_;
  std::map<std::string, std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>>> predicate_publishers_;

  std::shared_ptr<rclcpp::TimerBase> step_timer_;
  rclcpp::Parameter period_;
  rclcpp::Parameter has_tf_listener_;
  rclcpp::Parameter has_tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

template<class NodeT>
ComponentInterface<NodeT>::ComponentInterface(
    const rclcpp::NodeOptions& options, modulo_new_core::communication::PublisherType publisher_type
) :
    rclcpp::Node(utilities::parse_node_name(options, "ComponentInterface"), options), publisher_type_(publisher_type) {
  this->declare_parameter("period", 2.0);
  this->declare_parameter("has_tf_listener", false);
  this->declare_parameter("has_tf_broadcaster", false);

  // here I need to do typename NodeT:: because ParameterMap also has get_parameter
  period_ = NodeT::get_parameter("period");
  has_tf_listener_ = NodeT::get_parameter("has_tf_listener");
  has_tf_broadcaster_ = NodeT::get_parameter("has_tf_broadcaster");

  if (has_tf_listener_.as_bool()) {
    this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);
  }
  if (has_tf_broadcaster_.as_bool()) {
    this->tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->shared_from_this());
  }

  this->step_timer_ = this->create_wall_timer(
      std::chrono::nanoseconds(static_cast<int64_t>(this->period_.as_double() * 1e9)), [this] { this->step(); }
  );
}

template<class NodeT>
void ComponentInterface<NodeT>::step() {
  for (const auto& predicate: this->predicates_) {
    std_msgs::msg::Bool msg;
    msg.data = this->get_predicate(predicate.first);
    auto predicate_iterator = this->predicate_publishers_.find(predicate.first);
    if (predicate_iterator == this->predicate_publishers_.end()) {
      // TODO throw here
      RCLCPP_FATAL(this->get_logger(), "no publisher for predicate found");
      return;
    }
    predicate_publishers_.at(predicate.first)->publish(msg);
  }
}

template<class NodeT>
std::string ComponentInterface<NodeT>::generate_predicate_topic(const std::string& predicate_name) const {
  return "/predicates/" + std::string(this->get_name()) + "/" + predicate_name;
}

template<class NodeT>
void
ComponentInterface<NodeT>::add_variant_predicate(const std::string& name, const utilities::PredicateVariant& predicate) {
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

template<class NodeT>
void ComponentInterface<NodeT>::add_predicate(const std::string& name, bool predicate) {
  this->add_variant_predicate(name, utilities::PredicateVariant(predicate));
}

template<class NodeT>
void ComponentInterface<NodeT>::add_predicate(
    const std::string& name, const std::function<bool(void)>& predicate
) {
  this->add_variant_predicate(name, utilities::PredicateVariant(predicate));
}

template<class NodeT, typename PubT>
bool ComponentInterface<NodeT, PubT>::get_predicate(const std::string& predicate_name) const {
  auto predicate_iterator = this->predicates_.find(predicate_name);
  if (predicate_iterator == this->predicates_.end()) {
    throw exceptions::PredicateNotFoundException(predicate_name);
  }
  // try to get the value from the variant as a bool
  auto* ptr_value = std::get_if<bool>(&predicate_iterator->second);
  if (ptr_value) {
    return *ptr_value;
  }
  // if previous check failed, it means the variant is actually a callback function
  auto callback_function = std::get<std::function<bool(void)>>(predicate_iterator->second);
  return (callback_function)();
}

template<class NodeT>
void ComponentInterface<NodeT>::set_variant_predicate(
    const std::string& name, const utilities::PredicateVariant& predicate
) {
  auto predicate_iterator = this->predicates_.find(name);
  if (predicate_iterator == this->predicates_.end()) {
    throw exceptions::PredicateNotFoundException(name);
  }
  this->predicates_.at(name) = predicate;
}

template<class NodeT>
void ComponentInterface<NodeT>::set_predicate(const std::string& name, bool predicate) {
  this->set_variant_predicate(name, utilities::PredicateVariant(predicate));
}

template<class NodeT>
void ComponentInterface<NodeT>::set_predicate(
    const std::string& name, const std::function<bool(void)>& predicate
) {
  this->set_variant_predicate(name, utilities::PredicateVariant(predicate));
}

template<class NodeT>
void ComponentInterface<NodeT>::send_transform(const state_representation::CartesianPose& transform) {
  // TODO: throw here?
  if (this->tf_broadcaster_ == nullptr) {
    RCLCPP_FATAL(this->get_logger(), "No tf broadcaster");
  }
  geometry_msgs::msg::TransformStamped tf_message;
  modulo_new_core::translators::write_msg(tf_message, transform, this->get_clock()->now());
  this->tf_broadcaster_->sendTransform(tf_message);
}

template<class NodeT>
state_representation::CartesianPose ComponentInterface<NodeT>::lookup_transform(
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
