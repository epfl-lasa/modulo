#pragma once

#include <thread>

#include <rclcpp/node.hpp>

#include "modulo_components/ComponentInterface.hpp"
#include "modulo_components/utilities/utilities.hpp"
#include "modulo_new_core/EncodedState.hpp"

namespace modulo_components {

/**
 * @brief TODO
 */
class Component : public ComponentInterface<rclcpp::Node> {
public:
  friend class ComponentPublicInterface;

  /**
   * @brief Constructor from node options.
   * @param node_options Node options as used in ROS2 Node
   */
  explicit Component(const rclcpp::NodeOptions& node_options, bool start_thread = true);

  /**
   * @brief Virtual default destructor.
   */
  virtual ~Component() = default;

protected:
  /**
   * @brief Start the execution thread.
   */
  void start_thread();

  /**
   * @brief Execute the component logic. To be redefined in derived classes.
   * @return True, if the execution was successful, false otherwise
   */
  virtual bool execute();

  /**
   * @brief Add and configure an output signal of the component.
   * @tparam DataT Type of the data pointer
   * @param signal_name Name of the output signal
   * @param data Data to transmit on the output signal
   * @param fixed_topic If true, the topic name of the output signal is fixed
   * @param default_topic If set, the default value for the topic name to use
   */
  template<typename DataT>
  void add_output(
      const std::string& signal_name, const std::shared_ptr<DataT>& data, bool fixed_topic = false,
      const std::string& default_topic = ""
  );

private:
  /**
   * @brief Step function that is called periodically and publishes predicates,
   * outputs, and evaluates daemon callbacks.
   */
  void step() override;

  /**
   * @brief Run the execution function in a try catch block and
   * set the predicates according to the outcome of the execution.
   */
  void run();

  using rclcpp::Node::create_publisher;
  using ComponentInterface<rclcpp::Node>::create_output;
  using ComponentInterface<rclcpp::Node>::inputs_;
  using ComponentInterface<rclcpp::Node>::outputs_;
  using ComponentInterface<rclcpp::Node>::qos_;
  using ComponentInterface<rclcpp::Node>::publish_predicates;
  using ComponentInterface<rclcpp::Node>::publish_outputs;
  using ComponentInterface<rclcpp::Node>::evaluate_periodic_callbacks;
  using ComponentInterface<rclcpp::Node>::activate_tf_broadcaster;
  using ComponentInterface<rclcpp::Node>::deactivate_tf_broadcaster;

  std::thread run_thread_; ///< The execution thread of the component
  bool started_; ///< Flag that indicates if execution has started or not
};

template<typename DataT>
inline void Component::add_output(
    const std::string& signal_name, const std::shared_ptr<DataT>& data, bool fixed_topic,
    const std::string& default_topic
) {
  using namespace modulo_new_core::communication;
  try {
    std::string parsed_signal_name = utilities::parse_signal_name(signal_name);
    this->create_output(parsed_signal_name, data, fixed_topic, default_topic);
    auto topic_name = this->get_parameter_value<std::string>(parsed_signal_name + "_topic");
    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Adding output '" << signal_name << "' with topic name '" << topic_name << "'.");
    auto message_pair = this->outputs_.at(parsed_signal_name)->get_message_pair();
    switch (message_pair->get_type()) {
      case MessageType::BOOL: {
        auto publisher = this->create_publisher<std_msgs::msg::Bool>(topic_name, this->qos_);
        this->outputs_.at(parsed_signal_name) =
            std::make_shared<PublisherHandler<rclcpp::Publisher<std_msgs::msg::Bool>, std_msgs::msg::Bool>>(
                PublisherType::PUBLISHER, publisher
            )->create_publisher_interface(message_pair);
        break;
      }
      case MessageType::FLOAT64: {
        auto publisher = this->create_publisher<std_msgs::msg::Float64>(topic_name, this->qos_);
        this->outputs_.at(parsed_signal_name) =
            std::make_shared<PublisherHandler<rclcpp::Publisher<std_msgs::msg::Float64>, std_msgs::msg::Float64>>(
                PublisherType::PUBLISHER, publisher
            )->create_publisher_interface(message_pair);
        break;
      }
      case MessageType::FLOAT64_MULTI_ARRAY: {
        auto publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic_name, this->qos_);
        this->outputs_.at(parsed_signal_name) =
            std::make_shared<PublisherHandler<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>,
                                              std_msgs::msg::Float64MultiArray>>(
                PublisherType::PUBLISHER, publisher
            )->create_publisher_interface(message_pair);
        break;
      }
      case MessageType::INT32: {
        auto publisher = this->create_publisher<std_msgs::msg::Int32>(topic_name, this->qos_);
        this->outputs_.at(parsed_signal_name) =
            std::make_shared<PublisherHandler<rclcpp::Publisher<std_msgs::msg::Int32>, std_msgs::msg::Int32>>(
                PublisherType::PUBLISHER, publisher
            )->create_publisher_interface(message_pair);
        break;
      }
      case MessageType::STRING: {
        auto publisher = this->create_publisher<std_msgs::msg::String>(topic_name, this->qos_);
        this->outputs_.at(parsed_signal_name) =
            std::make_shared<PublisherHandler<rclcpp::Publisher<std_msgs::msg::String>, std_msgs::msg::String>>(
                PublisherType::PUBLISHER, publisher
            )->create_publisher_interface(message_pair);
        break;
      }
      case MessageType::ENCODED_STATE: {
        auto publisher = this->create_publisher<modulo_new_core::EncodedState>(topic_name, this->qos_);
        this->outputs_.at(parsed_signal_name) =
            std::make_shared<PublisherHandler<rclcpp::Publisher<modulo_new_core::EncodedState>,
                                              modulo_new_core::EncodedState>>(
                PublisherType::PUBLISHER, publisher
            )->create_publisher_interface(message_pair);
        break;
      }
    }
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to add output '" << signal_name << "': " << ex.what());
  }
}
}// namespace modulo_components
