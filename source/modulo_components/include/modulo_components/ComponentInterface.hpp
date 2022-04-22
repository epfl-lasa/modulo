#pragma once

#include <rclcpp/parameter.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <state_representation/parameters/ParameterMap.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>

#include <modulo_new_core/communication/MessagePair.hpp>
#include <modulo_new_core/communication/PublisherHandler.hpp>
#include <modulo_new_core/communication/PublisherType.hpp>
#include <modulo_new_core/translators/message_readers.hpp>
#include <modulo_new_core/translators/message_writers.hpp>
#include <modulo_new_core/translators/parameter_translators.hpp>

#include "modulo_components/utilities/utilities.hpp"
#include "modulo_components/utilities/predicate_variant.hpp"

namespace modulo_components {

template<class NodeT>
class ComponentInterface : public NodeT {
  friend class ComponentInterfaceTest;

public:
  friend class ComponentInterfacePublicInterface;

  /**
   * @brief Constructor from node options
   * @param node_options node options as used in ROS2 Node
   */
  explicit ComponentInterface(
      const rclcpp::NodeOptions& node_options, modulo_new_core::communication::PublisherType publisher_type
  );

protected:
  /**
   * @brief Add a parameter.
   * @details This method stores a pointer reference to an existing Parameter object in the local parameter map
   * and declares the equivalent ROS parameter on the ROS interface.
   * @param parameter A ParameterInterface pointer to a Parameter instance.
   */
  void add_parameter(
      const std::shared_ptr<state_representation::ParameterInterface>& parameter, const std::string& description,
      bool read_only = false
  );

  /**
   * @brief Add a parameter.
   * @details This method creates a new Parameter object instance to reference in the local parameter map
   * and declares the equivalent ROS parameter on the ROS interface.
   * @tparam T The type of the parameter
   * @param name The name of the parameter
   * @param value The value of the parameter
   */
  template<typename T>
  void add_parameter(const std::string& name, const T& value, const std::string& description, bool read_only = false);

  /**
   * @brief Get a parameter by name.
   * @param name The name of the parameter
   * @return The ParameterInterface pointer to a Parameter instance
   */
  [[nodiscard]] std::shared_ptr<state_representation::ParameterInterface> get_parameter(const std::string& name) const;

  /**
   * @brief Get a parameter value by name.
   * @tparam T The type of the parameter
   * @param name The name of the parameter
   * @return The value of the parameter
   */
  template<typename T>
  T get_parameter_value(const std::string& name) const;

  /**
   * @brief Set the value of a parameter.
   * @details The parameter must have been previously declared. This method preserves the reference
   * to the original Parameter instance
   * @tparam T The type of the parameter
   * @param name The name of the parameter
   * @return The value of the parameter
   */
  template<typename T>
  void set_parameter_value(const std::string& name, const T& value);

  /**
   * @brief Parameter validation function to be redefined by derived Component classes.
   * @details This method is automatically invoked whenever the ROS interface tried to modify a parameter.
   * Validation and sanitization can be performed by reading or writing the value of the parameter through the
   * ParameterInterface pointer, depending on the parameter name and desired component behaviour. If the validation
   * returns true, the updated parameter value (including any modifications) is applied. If the validation returns
   * false, any changes to the parameter are discarded and the parameter value is not changed.
   * @param parameter A ParameterInterface pointer to a Parameter instance
   * @return The validation result
   */
  virtual bool validate_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter);

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
   * @brief Get the logical value of a predicate.
   * @details If the predicate is not found or the callable function fails, the return value is false.
   * @param predicate_name the name of the predicate to retrieve from the
   * map of predicates
   * @return the value of the predicate as a boolean
   */
  [[nodiscard]] bool get_predicate(const std::string& predicate_name);

  /**
   * @brief Set the value of the predicate given as parameter, if the predicate is not found does not do anything
   * @param predicate_name the name of the predicate to retrieve from the
   * map of predicates
   * @param predicate_value the new value of the predicate
   */
  void set_predicate(const std::string& predicate_name, bool predicate_value);

  /**
   * @brief Set the value of the predicate given as parameter, if the predicate is not found does not do anything
   * @param predicate_name the name of the predicate to retrieve from the
   * map of predicates
   * @param predicate_function the function to call that returns the value of the predicate
   */
  void set_predicate(const std::string& predicate_name, const std::function<bool(void)>& predicate_function);

  /**
   * @brief Configure a transform broadcaster.
   */
  void add_tf_broadcaster();

  /**
   * @brief Configure a transform buffer and listener.
   */
  void add_tf_listener();

  template<typename DataT>
  void create_output(const std::string& signal_name, const std::shared_ptr<DataT>& data, bool fixed_topic = false);

  void send_transform(const state_representation::CartesianPose& transform);

  [[nodiscard]] state_representation::CartesianPose
  lookup_transform(const std::string& frame_name, const std::string& reference_frame_name = "world") const;

  std::map<std::string, std::shared_ptr<modulo_new_core::communication::PublisherInterface>> outputs_;

  rclcpp::QoS qos_ = rclcpp::QoS(10);

private:
  /**
   * @brief Callback function to validate and update parameters on change.
   * @param parameters The new parameter objects provided by the ROS interface
   * @return The result of the validation
   */
  rcl_interfaces::msg::SetParametersResult on_set_parameters_callback(const std::vector<rclcpp::Parameter>& parameters);

  [[nodiscard]] std::string generate_predicate_topic(const std::string& predicate_name) const;

  void add_variant_predicate(const std::string& name, const utilities::PredicateVariant& predicate);

  void set_variant_predicate(const std::string& name, const utilities::PredicateVariant& predicate);

  void step();

  modulo_new_core::communication::PublisherType publisher_type_;

  std::map<std::string, utilities::PredicateVariant> predicates_;
  std::map<std::string, std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>>> predicate_publishers_;

  state_representation::ParameterMap parameter_map_;
  std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> parameter_cb_handle_;

  std::shared_ptr<rclcpp::TimerBase> step_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

template<class NodeT>
ComponentInterface<NodeT>::ComponentInterface(
    const rclcpp::NodeOptions& options, modulo_new_core::communication::PublisherType publisher_type
) :
    NodeT(utilities::parse_node_name(options, "ComponentInterface"), options), publisher_type_(publisher_type) {
  // register the parameter change callback handler
  parameter_cb_handle_ = NodeT::add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters) -> rcl_interfaces::msg::SetParametersResult {
        return this->on_set_parameters_callback(parameters);
      });
  this->add_parameter("period", 1.0, "The time interval in seconds for all periodic callbacks", true);

  this->step_timer_ = this->create_wall_timer(
      std::chrono::nanoseconds(static_cast<int64_t>(this->get_parameter_value<double>("period") * 1e9)),
      [this] { this->step(); });
}

template<class NodeT>
void ComponentInterface<NodeT>::step() {
  for (const auto& predicate: this->predicates_) {
    std_msgs::msg::Bool msg;
    msg.data = this->get_predicate(predicate.first);
    auto predicate_iterator = this->predicate_publishers_.find(predicate.first);
    if (predicate_iterator == this->predicate_publishers_.end()) {
      RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                   "No publisher for predicate " << predicate.first << " found.");
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
void ComponentInterface<NodeT>::add_variant_predicate(
    const std::string& name, const utilities::PredicateVariant& predicate
) {
  if (this->predicates_.find(name) != this->predicates_.end()) {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Predicate " << name << " already exists, overwriting.");
    this->predicates_.at(name) = predicate;
  } else {
    this->predicates_.insert(std::make_pair(name, predicate));
    this->predicate_publishers_.insert(
        std::make_pair(
            name, this->template create_publisher<std_msgs::msg::Bool>(this->generate_predicate_topic(name), 10)));
  }
}

template<class NodeT>
template<typename T>
void ComponentInterface<NodeT>::add_parameter(
    const std::string& name, const T& value, const std::string& description, bool read_only
) {
  this->add_parameter(state_representation::make_shared_parameter(name, value), description, read_only);
}

template<class NodeT>
template<typename T>
T ComponentInterface<NodeT>::get_parameter_value(const std::string& name) const {
  return this->parameter_map_.template get_parameter_value<T>(name);
}

template<class NodeT>
void ComponentInterface<NodeT>::add_parameter(
    const std::shared_ptr<state_representation::ParameterInterface>& parameter, const std::string& description,
    bool read_only
) {
  auto ros_param = modulo_new_core::translators::write_parameter(parameter);
  if (!NodeT::has_parameter(parameter->get_name())) {
    parameter_map_.set_parameter(parameter);
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = read_only;
    NodeT::declare_parameter(parameter->get_name(), ros_param.get_parameter_value(), descriptor);
  } else {
    NodeT::set_parameter(ros_param);
  }
}

template<class NodeT>
std::shared_ptr<state_representation::ParameterInterface>
ComponentInterface<NodeT>::get_parameter(const std::string& name) const {
  return this->parameter_map_.get_parameter(name);
}

template<class NodeT>
template<typename T>
void ComponentInterface<NodeT>::set_parameter_value(const std::string& name, const T& value) {
  rcl_interfaces::msg::SetParametersResult result = NodeT::set_parameter(
      modulo_new_core::translators::write_parameter(state_representation::make_shared_parameter(name, value)));
  if (!result.successful) {
    throw state_representation::exceptions::InvalidParameterException(result.reason);
  }
}

template<class NodeT>
bool ComponentInterface<NodeT>::validate_parameter(
    const std::shared_ptr<state_representation::ParameterInterface>&
) {
  return true;
}

template<class NodeT>
rcl_interfaces::msg::SetParametersResult
ComponentInterface<NodeT>::on_set_parameters_callback(const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto& ros_parameter: parameters) {
    try {
      // get the associated parameter interface by name
      auto parameter = parameter_map_.get_parameter(ros_parameter.get_name());

      // convert the ROS parameter into a ParameterInterface without modifying the original
      auto new_parameter = modulo_new_core::translators::read_parameter_const(ros_parameter, parameter);
      if (!validate_parameter(new_parameter)) {
        result.successful = false;
        result.reason += "Parameter " + ros_parameter.get_name() + " could not be set! ";
      } else {
        // update the value of the parameter in the map
        modulo_new_core::translators::copy_parameter_value(new_parameter, parameter);
      }
    } catch (const std::exception& ex) {
      result.successful = false;
      result.reason += ex.what();
    }
  }
  return result;
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

template<class NodeT>
bool ComponentInterface<NodeT>::get_predicate(const std::string& predicate_name) {
  auto predicate_iterator = this->predicates_.find(predicate_name);
  // if there is no predicate with that name simply return false with an error message
  if (predicate_iterator == this->predicates_.end()) {
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                 "Predicate " << predicate_name << " does not exists, returning false.");
    return false;
  }
  // try to get the value from the variant as a bool
  auto* ptr_value = std::get_if<bool>(&predicate_iterator->second);
  if (ptr_value) {
    return *ptr_value;
  }
  // if previous check failed, it means the variant is actually a callback function
  auto callback_function = std::get<std::function<bool(void)>>(predicate_iterator->second);
  bool value = false;
  try {
    value = (callback_function)();
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                 "Error while evaluating the callback function: " << e.what());
  }
  return value;
}

template<class NodeT>
void ComponentInterface<NodeT>::set_variant_predicate(
    const std::string& name, const utilities::PredicateVariant& predicate
) {
  auto predicate_iterator = this->predicates_.find(name);
  if (predicate_iterator == this->predicates_.end()) {
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                 "Cannot set predicate " << name << " with a new value because it does not exist.");
    return;
  }
  predicate_iterator->second = predicate;
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
void ComponentInterface<NodeT>::add_tf_broadcaster() {
  this->tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->shared_from_this());
}

template<class NodeT>
void ComponentInterface<NodeT>::add_tf_listener() {
  this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);
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

template<class NodeT>
template<typename DataT>
inline void ComponentInterface<NodeT>::create_output(
    const std::string& signal_name, const std::shared_ptr<DataT>& data, bool fixed_topic
) {
  using namespace modulo_new_core::communication;
  auto message_pair = make_shared_message_pair(data, this->get_clock());
  this->outputs_.insert(
      std::make_pair(
          signal_name, std::make_shared<PublisherInterface>(this->publisher_type_, message_pair)));
  // TODO is default okay
  this->add_parameter(
      signal_name + "_topic", "~/" + signal_name, "Output topic name of signal '" + signal_name + "'", fixed_topic
  );
}

}// namespace modulo_components
