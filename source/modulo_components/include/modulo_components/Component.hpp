#pragma once

#include <rclcpp/node.hpp>

#include "modulo_components/ComponentInterface.hpp"
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
  template<typename DataT>
  void add_output(const std::string& signal_name, const std::shared_ptr<DataT>& data, bool fixed_topic = false);
};

template<typename DataT>
void Component::add_output(const std::string& signal_name, const std::shared_ptr<DataT>& data, bool fixed_topic) {
  using namespace modulo_new_core::communication;
  // TODO need to catch exceptions?
  // TODO parse signal name
  this->create_output(signal_name, data, fixed_topic);
  // TODO hardcoded _topic
  auto topic_name = this->get_parameter_value<std::string>(signal_name + "_topic");
  auto message_pair = this->outputs_.at(signal_name)->get_message_pair();
  switch (message_pair->get_type()) {
    case MessageType::BOOL: {
      auto publisher = this->create_publisher<std_msgs::msg::Bool>(topic_name, this->qos_);
      this->outputs_.at(signal_name) =
          std::make_shared<PublisherHandler<rclcpp::Publisher<std_msgs::msg::Bool>, std_msgs::msg::Bool>>(
              PublisherType::PUBLISHER, publisher
          )->create_publisher_interface(message_pair);
      break;
    }
    case MessageType::FLOAT64: {
      auto publisher = this->create_publisher<std_msgs::msg::Float64>(topic_name, this->qos_);
      this->outputs_.at(signal_name) =
          std::make_shared<PublisherHandler<rclcpp::Publisher<std_msgs::msg::Float64>, std_msgs::msg::Float64>>(
              PublisherType::PUBLISHER, publisher
          )->create_publisher_interface(message_pair);
      break;
    }
    case MessageType::FLOAT64_MULTI_ARRAY: {
      auto publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic_name, this->qos_);
      this->outputs_.at(signal_name) =
          std::make_shared<PublisherHandler<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>,
                                            std_msgs::msg::Float64MultiArray>>(
              PublisherType::PUBLISHER, publisher
          )->create_publisher_interface(message_pair);
      break;
    }
    case MessageType::INT32: {
      auto publisher = this->create_publisher<std_msgs::msg::Int32>(topic_name, this->qos_);
      this->outputs_.at(signal_name) =
          std::make_shared<PublisherHandler<rclcpp::Publisher<std_msgs::msg::Int32>, std_msgs::msg::Int32>>(
              PublisherType::PUBLISHER, publisher
          )->create_publisher_interface(message_pair);
      break;
    }
    case MessageType::STRING: {
      auto publisher = this->create_publisher<std_msgs::msg::String>(topic_name, this->qos_);
      this->outputs_.at(signal_name) =
          std::make_shared<PublisherHandler<rclcpp::Publisher<std_msgs::msg::String>, std_msgs::msg::String>>(
              PublisherType::PUBLISHER, publisher
          )->create_publisher_interface(message_pair);
      break;
    }
    case MessageType::ENCODED_STATE: {
      auto publisher = this->create_publisher<modulo_new_core::EncodedState>(topic_name, this->qos_);
      this->outputs_.at(signal_name) =
          std::make_shared<PublisherHandler<rclcpp::Publisher<modulo_new_core::EncodedState>,
                                            modulo_new_core::EncodedState>>(
              PublisherType::PUBLISHER, publisher
          )->create_publisher_interface(message_pair);
      break;
    }
  }
}

}