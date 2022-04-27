#pragma once

#include <rclcpp/node.hpp>

#include "modulo_components/ComponentInterface.hpp"
#include "modulo_components/utilities/utilities.hpp"
#include "modulo_new_core/EncodedState.hpp"

namespace modulo_components {

class Component : public ComponentInterface<rclcpp::Node> {
public:
  friend class ComponentPublicInterface;

  /**
   * @brief Constructor from node options
   * @param node_options node options as used in ROS2 Node
   */
  explicit Component(const rclcpp::NodeOptions& node_options);

protected:
  /**
   * @brief Add and configure an output signal of the component.
   * @tparam DataT Type of the data pointer
   * @param signal_name Name of the output signal
   * @param data Data to transmit on the output signal
   * @param fixed_topic If true, the topic name of the output signal is fixed
   */
  template<typename DataT>
  void add_output(
      const std::string& signal_name, const std::shared_ptr<DataT>& data, bool fixed_topic = false,
      const std::string& default_topic = ""
  );

private:
  using ComponentInterface<rclcpp::Node>::create_output;
  using ComponentInterface<rclcpp::Node>::outputs_;
  using ComponentInterface<rclcpp::Node>::qos_;
};

template<typename DataT>
void Component::add_output(
    const std::string& signal_name, const std::shared_ptr<DataT>& data, bool fixed_topic,
    const std::string& default_topic
) {
  using namespace modulo_new_core::communication;
  try {
    std::string parsed_signal_name = utilities::parse_signal_name(signal_name);
    this->create_output(parsed_signal_name, data, fixed_topic, default_topic);
    auto topic_name = this->get_parameter_value<std::string>(parsed_signal_name + "_topic");
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

}