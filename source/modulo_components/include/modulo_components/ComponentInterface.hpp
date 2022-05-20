#pragma once

#include <rclcpp/parameter.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <state_representation/parameters/ParameterMap.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>

#include <modulo_new_core/communication/MessagePair.hpp>
#include <modulo_new_core/communication/PublisherHandler.hpp>
#include <modulo_new_core/communication/PublisherType.hpp>
#include <modulo_new_core/communication/SubscriptionHandler.hpp>
#include <modulo_new_core/translators/message_readers.hpp>
#include <modulo_new_core/translators/message_writers.hpp>
#include <modulo_new_core/translators/parameter_translators.hpp>

#include "modulo_components/exceptions/AddSignalException.hpp"
#include "modulo_components/exceptions/ComponentParameterException.hpp"
#include "modulo_components/exceptions/LookupTransformException.hpp"
#include "modulo_components/utilities/utilities.hpp"
#include "modulo_components/utilities/predicate_variant.hpp"

namespace modulo_components {

template<class NodeT>
class ComponentInterfacePublicInterface;

template<class NodeT>
class ComponentInterface : private NodeT {
public:
  friend class ComponentInterfacePublicInterface<rclcpp::Node>;
  friend class ComponentInterfacePublicInterface<rclcpp_lifecycle::LifecycleNode>;

  /**
   * @brief Constructor from node options.
   * @param node_options Node options as used in ROS2 Node / LifecycleNode
   */
  explicit ComponentInterface(
      const rclcpp::NodeOptions& node_options, modulo_new_core::communication::PublisherType publisher_type
  );

  /**
   * @brief Virtual default destructor.
   */
  virtual ~ComponentInterface() = default;

  using NodeT::get_node_base_interface;
  using NodeT::get_name;
  using NodeT::get_clock;
  using NodeT::get_logger;

protected:
  /**
   * @brief Step function that is called periodically.
   */
  virtual void step();

  /**
   * @brief Add a parameter.
   * @details This method stores a pointer reference to an existing Parameter object in the local parameter map
   * and declares the equivalent ROS parameter on the ROS interface.
   * @param parameter A ParameterInterface pointer to a Parameter instance
   * @param description The description of the parameter
   * @param read_only If true, the value of the parameter cannot be changed after declaration
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
   * @param description The description of the parameter
   * @param read_only If true, the value of the parameter cannot be changed after declaration
   */
  template<typename T>
  void add_parameter(const std::string& name, const T& value, const std::string& description, bool read_only = false);

  /**
   * @brief Get a parameter by name.
   * @param name The name of the parameter
   * @throws ComponentParameterException if the parameter could not be found
   * @return The ParameterInterface pointer to a Parameter instance
   */
  [[nodiscard]] std::shared_ptr<state_representation::ParameterInterface> get_parameter(const std::string& name) const;

  /**
   * @brief Get a parameter value by name.
   * @tparam T The type of the parameter
   * @param name The name of the parameter
   * @throws ComponentParameterException if the parameter value could not be accessed
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
   * @brief Add a predicate to the map of predicates.
   * @param predicate_name the name of the associated predicate
   * @param predicate_value the boolean value of the predicate
   */
  void add_predicate(const std::string& predicate_name, bool predicate_value);

  /**
   * @brief Add a predicate to the map of predicates based on a function to periodically call.
   * @param predicate_name the name of the associated predicate
   * @param predicate_function the function to call that returns the value of the predicate
   */
  void add_predicate(const std::string& predicate_name, const std::function<bool(void)>& predicate_function);

  /**
   * @brief Get the logical value of a predicate.
   * @details If the predicate is not found or the callable function fails, the return value is false.
   * @param predicate_name the name of the predicate to retrieve from the map of predicates
   * @return the value of the predicate as a boolean
   */
  [[nodiscard]] bool get_predicate(const std::string& predicate_name);

  /**
   * @brief Set the value of the predicate given as parameter, if the predicate is not found does not do anything.
   * @param predicate_name the name of the predicate to retrieve from the map of predicates
   * @param predicate_value the new value of the predicate
   */
  void set_predicate(const std::string& predicate_name, bool predicate_value);

  /**
   * @brief Set the value of the predicate given as parameter, if the predicate is not found does not do anything.
   * @param predicate_name the name of the predicate to retrieve from the map of predicates
   * @param predicate_function the function to call that returns the value of the predicate
   */
  void set_predicate(const std::string& predicate_name, const std::function<bool(void)>& predicate_function);

  /**
   * @brief Add and configure an input signal of the component.
   * @tparam DataT Type of the data pointer
   * @param signal_name Name of the input signal
   * @param data Data to receive on the input signal
   * @param fixed_topic If true, the topic name of the input signal is fixed
   * @param default_topic If set, the default value for the topic name to use
   */
  template<typename DataT>
  void add_input(
      const std::string& signal_name, const std::shared_ptr<DataT>& data, bool fixed_topic = false,
      const std::string& default_topic = ""
  );

  /**
   * @brief Add and configure an input signal of the component.
   * @tparam MsgT The ROS message type of the subscription
   * @param signal_name Name of the input signal
   * @param callback The callback to use for the subscription
   * @param fixed_topic If true, the topic name of the input signal is fixed
   * @param default_topic If set, the default value for the topic name to use
   */
  template<typename MsgT>
  void add_input(
      const std::string& signal_name, const std::function<void(const std::shared_ptr<MsgT>)>& callback,
      bool fixed_topic = false, const std::string& default_topic = ""
  );

  /**
   * @brief Add a periodic callback function.
   * @details The provided function is evaluated periodically at the component step period.
   * @param name The name of the callback
   * @param callback The callback function that is evaluated periodically
   */
  void add_periodic_function(const std::string& name, const std::function<void(void)>& callback);

  /**
   * @brief Configure a transform broadcaster.
   */
  void add_tf_broadcaster();

  /**
   * @brief Configure a transform buffer and listener.
   */
  void add_tf_listener();

  /**
   * @brief Helper function to parse the signal name and add an unconfigured PublisherInterface to the map of outputs.
   * @tparam DataT Type of the data pointer
   * @param signal_name Name of the output signal
   * @param data Data to transmit on the output signal
   * @param fixed_topic If true, the topic name of the output signal is fixed
   * @throws AddSignalException if the output could not be created (empty name, already registered)
   */
  template<typename DataT>
  void create_output(
      const std::string& signal_name, const std::shared_ptr<DataT>& data, bool fixed_topic = false,
      const std::string& default_topic = ""
  );

  /**
   * @brief Getter of the Quality of Service attribute.
   * @return The Quality of Service attribute
   */
  [[nodiscard]] rclcpp::QoS get_qos() const;

  /**
   * @brief Set the Quality of Service for ROS publishers and subscribers.
   * @param qos The desired Quality of Service
   */
  void set_qos(const rclcpp::QoS& qos);

  /**
   * @brief Send a transform to TF.
   * @param transform The transform to send
   */
  void send_transform(const state_representation::CartesianPose& transform);

  /**
   * @brief Look up a transform from TF.
   * @param frame_name The desired frame of the transform
   * @param reference_frame_name The desired reference frame of the transform
   * @param time_point The time at which the value of the transform is desired (default: 0, will get the latest)
   * @param duration How long to block the lookup call before failing
   * @throws LookupTransformException if TF buffer/listener are unconfigured or if the lookupTransform call failed
   * @return If it exists, the requested transform
   */
  [[nodiscard]] state_representation::CartesianPose lookup_transform(
      const std::string& frame_name, const std::string& reference_frame_name = "world",
      const tf2::TimePoint& time_point = tf2::TimePoint(std::chrono::microseconds(0)),
      const tf2::Duration& duration = tf2::Duration(std::chrono::microseconds(10))) const;

  /**
   * @brief Helper function to publish all predicates.
   */
  void publish_predicates();

  /**
   * @brief Helper function to publish all output signals.
   */
  void publish_outputs();

  /**
   * @brief Helper function to evaluate all periodic function callbacks.
   */
  void evaluate_periodic_callbacks();

  /**
   * @brief Put the component in error state by setting the
   * 'in_error_state' predicate to true.
   */
  virtual void raise_error();

  using NodeT::create_publisher;

  std::map<std::string, std::shared_ptr<modulo_new_core::communication::PublisherInterface>>
      outputs_; ///< Map of outputs

  rclcpp::QoS qos_ = rclcpp::QoS(10); ///< Quality of Service for ROS publishers and subscribers

private:
  /**
   * @brief Callback function to validate and update parameters on change.
   * @param parameters The new parameter objects provided by the ROS interface
   * @return The result of the validation
   */
  rcl_interfaces::msg::SetParametersResult on_set_parameters_callback(const std::vector<rclcpp::Parameter>& parameters);

  /**
   * @brief Add a predicate to the map of predicates.
   * @param name The name of the predicate
   * @param predicate The predicate variant
   */
  void add_variant_predicate(const std::string& name, const utilities::PredicateVariant& predicate);

  /**
   * @brief Set the predicate given as parameter, if the predicate is not found does not do anything.
   * @param name The name of the predicate
   * @param predicate The predicate variant
   */
  void set_variant_predicate(const std::string& name, const utilities::PredicateVariant& predicate);

  modulo_new_core::communication::PublisherType
      publisher_type_; ///< Type of the output publishers (one of PUBLISHER, LIFECYCLE_PUBLISHER)

  std::map<std::string, utilities::PredicateVariant> predicates_; ///< Map of predicates
  std::map<std::string, std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>>>
      predicate_publishers_; ///< Map of predicate publishers
  std::map<std::string, std::shared_ptr<modulo_new_core::communication::SubscriptionInterface>> inputs_;

  std::map<std::string, std::function<void(void)>> periodic_callbacks_; ///< Map of periodic function callbacks

  state_representation::ParameterMap parameter_map_; ///< ParameterMap for handling parameters
  std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle>
      parameter_cb_handle_; ///< ROS callback function handle for setting parameters

  std::shared_ptr<rclcpp::TimerBase> step_timer_; ///< Timer for the step function
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_; ///< TF buffer
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_; ///< TF listener
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; ///< TF broadcaster
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
      }
  );
  this->add_parameter("period", 1.0, "The time interval in seconds for all periodic callbacks", true);

  this->add_predicate("in_error_state", false);

  this->step_timer_ = this->create_wall_timer(
      std::chrono::nanoseconds(static_cast<int64_t>(this->get_parameter_value<double>("period") * 1e9)),
      [this] { this->step(); }
  );
}

template<class NodeT>
inline void ComponentInterface<NodeT>::step() {}

template<class NodeT>
template<typename T>
inline void ComponentInterface<NodeT>::add_parameter(
    const std::string& name, const T& value, const std::string& description, bool read_only
) {
  if (name.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to add parameter: Provide a non empty string as a name.");
    return;
  }
  this->add_parameter(state_representation::make_shared_parameter(name, value), description, read_only);
}

template<class NodeT>
template<typename T>
inline T ComponentInterface<NodeT>::get_parameter_value(const std::string& name) const {
  try {
    return this->parameter_map_.template get_parameter_value<T>(name);
  } catch (const state_representation::exceptions::InvalidParameterException& ex) {
    throw exceptions::ComponentParameterException(
        "Failed to get parameter value of parameter '" + name + "': " + ex.what());
  }
}

template<class NodeT>
inline void ComponentInterface<NodeT>::add_parameter(
    const std::shared_ptr<state_representation::ParameterInterface>& parameter, const std::string& description,
    bool read_only
) {
  try {
    auto ros_param = modulo_new_core::translators::write_parameter(parameter);
    if (!NodeT::has_parameter(parameter->get_name())) {
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Adding parameter '" << parameter->get_name() << "'.");
      parameter_map_.set_parameter(parameter);
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.description = description;
      descriptor.read_only = read_only;
      NodeT::declare_parameter(parameter->get_name(), ros_param.get_parameter_value(), descriptor);
    } else {
      RCLCPP_DEBUG_STREAM(this->get_logger(),
                          "Parameter '" << parameter->get_name() << "' already exists, overwriting.");
      NodeT::set_parameter(ros_param);
    }
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to add parameter '" << parameter->get_name() << "': " << ex.what());
  }
}

template<class NodeT>
inline std::shared_ptr<state_representation::ParameterInterface>
ComponentInterface<NodeT>::get_parameter(const std::string& name) const {
  try {
    return this->parameter_map_.get_parameter(name);
  } catch (const state_representation::exceptions::InvalidParameterException& ex) {
    throw exceptions::ComponentParameterException("Failed to get parameter '" + name + "': " + ex.what());
  }
}

template<class NodeT>
template<typename T>
inline void ComponentInterface<NodeT>::set_parameter_value(const std::string& name, const T& value) {
  try {
    rcl_interfaces::msg::SetParametersResult result = NodeT::set_parameter(
        modulo_new_core::translators::write_parameter(state_representation::make_shared_parameter(name, value)));
    if (!result.successful) {
      RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                   "Failed to set parameter value of parameter '" << name << "': " << result.reason);
    }
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Failed to set parameter value of parameter '" << name << "': " << ex.what());
  }
}

template<class NodeT>
inline bool ComponentInterface<NodeT>::validate_parameter(
    const std::shared_ptr<state_representation::ParameterInterface>&
) {
  return true;
}

template<class NodeT>
inline rcl_interfaces::msg::SetParametersResult
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
inline void ComponentInterface<NodeT>::add_predicate(const std::string& name, bool predicate) {
  this->add_variant_predicate(name, utilities::PredicateVariant(predicate));
}

template<class NodeT>
inline void ComponentInterface<NodeT>::add_predicate(
    const std::string& name, const std::function<bool(void)>& predicate
) {
  this->add_variant_predicate(name, utilities::PredicateVariant(predicate));
}

template<class NodeT>
inline void ComponentInterface<NodeT>::add_variant_predicate(
    const std::string& name, const utilities::PredicateVariant& predicate
) {
  if (name.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to add predicate: Provide a non empty string as a name.");
    return;
  }
  if (this->predicates_.find(name) != this->predicates_.end()) {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Predicate '" << name << "' already exists, overwriting.");
  } else {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Adding predicate '" << name << "'.");
    this->predicate_publishers_.insert_or_assign(
        name, this->template create_publisher<std_msgs::msg::Bool>(
            utilities::generate_predicate_topic(this->get_name(), name), this->qos_
        ));
  }
  this->predicates_.insert_or_assign(name, predicate);
}

template<class NodeT>
inline bool ComponentInterface<NodeT>::get_predicate(const std::string& predicate_name) {
  auto predicate_iterator = this->predicates_.find(predicate_name);
  // if there is no predicate with that name simply return false with an error message
  if (predicate_iterator == this->predicates_.end()) {
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Failed to get predicate '" << predicate_name
                                                             << "': Predicate does not exists, returning false.");
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
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Failed to evaluate callback of predicate'" << predicate_name << "':" << ex.what());
  }
  return value;
}

template<class NodeT>
inline void ComponentInterface<NodeT>::set_variant_predicate(
    const std::string& name, const utilities::PredicateVariant& predicate
) {
  auto predicate_iterator = this->predicates_.find(name);
  if (predicate_iterator == this->predicates_.end()) {
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Failed to set predicate '" << name << "': Predicate does not exist.");
    return;
  }
  predicate_iterator->second = predicate;
}

template<class NodeT>
inline void ComponentInterface<NodeT>::set_predicate(const std::string& name, bool predicate) {
  this->set_variant_predicate(name, utilities::PredicateVariant(predicate));
}

template<class NodeT>
inline void ComponentInterface<NodeT>::set_predicate(
    const std::string& name, const std::function<bool(void)>& predicate
) {
  this->set_variant_predicate(name, utilities::PredicateVariant(predicate));
}

template<class NodeT>
template<typename DataT>
inline void ComponentInterface<NodeT>::add_input(
    const std::string& signal_name, const std::shared_ptr<DataT>& data, bool fixed_topic,
    const std::string& default_topic
) {
  using namespace modulo_new_core::communication;
  try {
    std::string parsed_signal_name = utilities::parse_signal_name(signal_name);
    if (parsed_signal_name.empty()) {
      throw exceptions::AddSignalException(
          "Failed to add input '" + signal_name + "': Parsed signal name is empty."
      );
    }
    if (this->inputs_.find(parsed_signal_name) != this->inputs_.end()) {
      throw exceptions::AddSignalException("Failed to add input '" + signal_name + "': Input already exists");
    }
    std::string topic_name = default_topic.empty() ? "~/" + parsed_signal_name : default_topic;
    this->add_parameter(
        parsed_signal_name + "_topic", topic_name, "Output topic name of signal '" + parsed_signal_name + "'",
        fixed_topic
    );
    topic_name = this->get_parameter_value<std::string>(parsed_signal_name + "_topic");
    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Adding input '" << signal_name << "' with topic name '" << topic_name << "'.");
    auto message_pair = make_shared_message_pair(data, this->get_clock());
    std::shared_ptr<SubscriptionInterface> subscription_interface;
    switch (message_pair->get_type()) {
      case MessageType::BOOL: {
        auto subscription_handler = std::make_shared<SubscriptionHandler<std_msgs::msg::Bool>>(message_pair);
        auto subscription = NodeT::template create_subscription<std_msgs::msg::Bool>(
            topic_name, this->qos_, subscription_handler->get_callback());
        subscription_interface = subscription_handler->create_subscription_interface(subscription);
        break;
      }
      case MessageType::FLOAT64: {
        auto subscription_handler = std::make_shared<SubscriptionHandler<std_msgs::msg::Float64>>(message_pair);
        auto subscription = NodeT::template create_subscription<std_msgs::msg::Float64>(
            topic_name, this->qos_, subscription_handler->get_callback());
        subscription_interface = subscription_handler->create_subscription_interface(subscription);
        break;
      }
      case MessageType::FLOAT64_MULTI_ARRAY: {
        auto subscription_handler =
            std::make_shared<SubscriptionHandler<std_msgs::msg::Float64MultiArray>>(message_pair);
        auto subscription = NodeT::template create_subscription<std_msgs::msg::Float64MultiArray>(
            topic_name, this->qos_, subscription_handler->get_callback());
        subscription_interface = subscription_handler->create_subscription_interface(subscription);
        break;
      }
      case MessageType::INT32: {
        auto subscription_handler = std::make_shared<SubscriptionHandler<std_msgs::msg::Int32>>(message_pair);
        auto subscription = NodeT::template create_subscription<std_msgs::msg::Int32>(
            topic_name, this->qos_, subscription_handler->get_callback());
        subscription_interface = subscription_handler->create_subscription_interface(subscription);
        break;
      }
      case MessageType::STRING: {
        auto subscription_handler = std::make_shared<SubscriptionHandler<std_msgs::msg::String>>(message_pair);
        auto subscription = NodeT::template create_subscription<std_msgs::msg::String>(
            topic_name, this->qos_, subscription_handler->get_callback());
        subscription_interface = subscription_handler->create_subscription_interface(subscription);
        break;
      }
      case MessageType::ENCODED_STATE: {
        auto subscription_handler = std::make_shared<SubscriptionHandler<modulo_new_core::EncodedState>>(message_pair);
        auto subscription = NodeT::template create_subscription<modulo_new_core::EncodedState>(
            topic_name, this->qos_, subscription_handler->get_callback());
        subscription_interface = subscription_handler->create_subscription_interface(subscription);
        break;
      }
    }
    this->inputs_.insert_or_assign(parsed_signal_name, subscription_interface);
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to add input '" << signal_name << "': " << ex.what());
  }
}

template<class NodeT>
template<typename MsgT>
inline void ComponentInterface<NodeT>::add_input(
    const std::string& signal_name, const std::function<void(const std::shared_ptr<MsgT>)>& callback, bool fixed_topic,
    const std::string& default_topic
) {
  using namespace modulo_new_core::communication;
  try {
    std::string parsed_signal_name = utilities::parse_signal_name(signal_name);
    if (parsed_signal_name.empty()) {
      throw exceptions::AddSignalException(
          "Failed to add input '" + signal_name + "': Parsed signal name is empty."
      );
    }
    if (this->inputs_.find(parsed_signal_name) != this->inputs_.end()) {
      throw exceptions::AddSignalException("Failed to add input '" + signal_name + "': Input already exists");
    }
    std::string topic_name = default_topic.empty() ? "~/" + parsed_signal_name : default_topic;
    this->add_parameter(
        parsed_signal_name + "_topic", topic_name, "Output topic name of signal '" + parsed_signal_name + "'",
        fixed_topic
    );
    topic_name = this->get_parameter_value<std::string>(parsed_signal_name + "_topic");
    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Adding input '" << signal_name << "' with topic name '" << topic_name << "'.");
    auto subscription = NodeT::template create_subscription<MsgT>(topic_name, this->qos_, callback);
    auto subscription_interface =
        std::make_shared<SubscriptionHandler<MsgT>>()->create_subscription_interface(subscription);
    this->inputs_.insert_or_assign(parsed_signal_name, subscription_interface);
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to add input '" << signal_name << "': " << ex.what());
  }
}

template<class NodeT>
inline void
ComponentInterface<NodeT>::add_periodic_function(const std::string& name, const std::function<void()>& callback) {
  if (name.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to add periodic function: Provide a non empty string as a name.");
    return;
  }
  if (this->periodic_callbacks_.find(name) != this->periodic_callbacks_.end()) {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Periodic function '" << name << "' already exists, overwriting.");
  } else {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Adding periodic function '" << name << "'.");
  }
  this->periodic_callbacks_.template insert_or_assign(name, callback);
}

template<class NodeT>
inline void ComponentInterface<NodeT>::add_tf_broadcaster() {
  if (this->tf_broadcaster_ == nullptr) {
    RCLCPP_DEBUG(this->get_logger(), "Adding TF broadcaster.");
    this->tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->shared_from_this());
  } else {
    RCLCPP_DEBUG(this->get_logger(), "TF broadcaster already exists.");
  }
}

template<class NodeT>
inline void ComponentInterface<NodeT>::add_tf_listener() {
  if (this->tf_buffer_ == nullptr || this->tf_listener_ == nullptr) {
    RCLCPP_DEBUG(this->get_logger(), "Adding TF buffer and listener.");
    this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);
  } else {
    RCLCPP_DEBUG(this->get_logger(), "TF buffer and listener already exist.");
  }
}

template<class NodeT>
inline void ComponentInterface<NodeT>::send_transform(const state_representation::CartesianPose& transform) {
  if (this->tf_broadcaster_ == nullptr) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock, 1000,
                          "Failed to send transform: No tf broadcaster configured.");
    return;
  }
  // TODO try catch here
  geometry_msgs::msg::TransformStamped tf_message;
  modulo_new_core::translators::write_msg(tf_message, transform, this->get_clock()->now());
  this->tf_broadcaster_->sendTransform(tf_message);
}

template<class NodeT>
inline state_representation::CartesianPose ComponentInterface<NodeT>::lookup_transform(
    const std::string& frame_name, const std::string& reference_frame_name, const tf2::TimePoint& time_point,
    const tf2::Duration& duration
) const {
  if (this->tf_buffer_ == nullptr || this->tf_listener_ == nullptr) {
    throw exceptions::LookupTransformException("Failed to lookup transform: To TF buffer / listener configured.");
  }
  geometry_msgs::msg::TransformStamped transform;
  state_representation::CartesianPose result(frame_name, reference_frame_name);
  try {
    transform = this->tf_buffer_->lookupTransform(reference_frame_name, frame_name, time_point, duration);
  } catch (const tf2::TransformException& ex) {
    throw exceptions::LookupTransformException(std::string("Failed to lookup transform: ").append(ex.what()));
  }
  modulo_new_core::translators::read_msg(result, transform);
  return result;
}

template<class NodeT>
inline void ComponentInterface<NodeT>::publish_predicates() {
  for (const auto& predicate: this->predicates_) {
    std_msgs::msg::Bool msg;
    msg.data = this->get_predicate(predicate.first);
    if (this->predicate_publishers_.find(predicate.first) == this->predicate_publishers_.end()) {
      RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                   "No publisher for predicate " << predicate.first << " found.");
      return;
    }
    predicate_publishers_.at(predicate.first)->publish(msg);
  }
}

template<class NodeT>
inline void ComponentInterface<NodeT>::publish_outputs() {
  for (const auto& [signal, publisher]: this->outputs_) {
    try {
      publisher->publish();
    } catch (const std::exception& ex) {
      RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                   "Failed to publish output '" << signal << "': " << ex.what());
    }
  }
}

template<class NodeT>
inline void ComponentInterface<NodeT>::evaluate_periodic_callbacks() {
  for (const auto& [name, callback]: this->periodic_callbacks_) {
    try {
      callback();
    } catch (const std::exception& ex) {
      RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                   "Failed to evaluate periodic function callback '" << name << "': " << ex.what());
    }
  }
}

template<class NodeT>
template<typename DataT>
inline void ComponentInterface<NodeT>::create_output(
    const std::string& signal_name, const std::shared_ptr<DataT>& data, bool fixed_topic,
    const std::string& default_topic
) {
  using namespace modulo_new_core::communication;
  if (signal_name.empty()) {
    throw exceptions::AddSignalException("Failed to add output: Provide a non empty string as a name.");
  }
  if (this->outputs_.find(signal_name) != this->outputs_.end()) {
    throw exceptions::AddSignalException("Output with name '" + signal_name + "' already exists");
  }
  auto message_pair = make_shared_message_pair(data, this->get_clock());
  this->outputs_.insert_or_assign(
      signal_name, std::make_shared<PublisherInterface>(this->publisher_type_, message_pair));
  std::string topic_name = default_topic.empty() ? "~/" + signal_name : default_topic;
  this->add_parameter(
      signal_name + "_topic", topic_name, "Output topic name of signal '" + signal_name + "'", fixed_topic
  );
}

template<class NodeT>
inline void ComponentInterface<NodeT>::raise_error() {
  RCLCPP_DEBUG(this->get_logger(), "raise_error called: Setting predicate 'in_error_state' to true.");
  this->set_predicate("in_error_state", true);
}

template<class NodeT>
inline rclcpp::QoS ComponentInterface<NodeT>::get_qos() const {
  return this->qos_;
}

template<class NodeT>
inline void ComponentInterface<NodeT>::set_qos(const rclcpp::QoS& qos) {
  this->qos_ = qos;
}

}// namespace modulo_components
