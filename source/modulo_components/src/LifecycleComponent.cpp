#include "modulo_components/LifecycleComponent.hpp"

using namespace modulo_new_core::communication;

namespace modulo_components {

LifecycleComponent::LifecycleComponent(const rclcpp::NodeOptions& node_options) :
    ComponentInterface<rclcpp_lifecycle::LifecycleNode>(node_options, PublisherType::LIFECYCLE_PUBLISHER) {}

void LifecycleComponent::step() {
  // TODO do this only if active and add on_step
//  this->publish_predicates();
//  this->publish_outputs();
//  this->evaluate_daemon_callbacks();
//  this->on_step();
}

bool LifecycleComponent::configure_outputs() {
  bool success = true;
  for (auto& [name, interface]: this->outputs_) {
    try {
      auto topic_name = this->get_parameter_value<std::string>(name + "_topic");
      RCLCPP_DEBUG_STREAM(this->get_logger(),
                          "Configuring output '" << name << "' with topic name '" << topic_name << "'.");
      auto message_pair = interface->get_message_pair();
      switch (message_pair->get_type()) {
        case MessageType::BOOL: {
          auto publisher = this->create_publisher<std_msgs::msg::Bool>(topic_name, this->qos_);
          interface = std::make_shared<PublisherHandler<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>,
                                                        std_msgs::msg::Bool>>(
              PublisherType::LIFECYCLE_PUBLISHER, publisher
          )->create_publisher_interface(message_pair);
          break;
        }
        case MessageType::FLOAT64: {
          auto publisher = this->create_publisher<std_msgs::msg::Float64>(topic_name, this->qos_);
          interface = std::make_shared<PublisherHandler<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>,
                                                        std_msgs::msg::Float64>>(
              PublisherType::LIFECYCLE_PUBLISHER, publisher
          )->create_publisher_interface(message_pair);
          break;
        }
        case MessageType::FLOAT64_MULTI_ARRAY: {
          auto publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(
              topic_name, this->qos_
          );
          interface =
              std::make_shared<PublisherHandler<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>,
                                                std_msgs::msg::Float64MultiArray>>(
                  PublisherType::LIFECYCLE_PUBLISHER, publisher
              )->create_publisher_interface(message_pair);
          break;
        }
        case MessageType::INT32: {
          auto publisher = this->create_publisher<std_msgs::msg::Int32>(topic_name, this->qos_);
          interface = std::make_shared<PublisherHandler<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>,
                                                        std_msgs::msg::Int32>>(
              PublisherType::LIFECYCLE_PUBLISHER, publisher
          )->create_publisher_interface(message_pair);
          break;
        }
        case MessageType::STRING: {
          auto publisher = this->create_publisher<std_msgs::msg::String>(topic_name, this->qos_);
          interface = std::make_shared<PublisherHandler<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>,
                                                        std_msgs::msg::String>>(
              PublisherType::LIFECYCLE_PUBLISHER, publisher
          )->create_publisher_interface(message_pair);
          break;
        }
        case MessageType::ENCODED_STATE: {
          auto publisher = this->create_publisher<modulo_new_core::EncodedState>(topic_name, this->qos_);
          interface =
              std::make_shared<PublisherHandler<rclcpp_lifecycle::LifecyclePublisher<modulo_new_core::EncodedState>,
                                                modulo_new_core::EncodedState>>(
                  PublisherType::LIFECYCLE_PUBLISHER, publisher
              )->create_publisher_interface(message_pair);
          break;
        }
      }
    } catch (const std::exception& ex) {
      success = false;
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to configure output '" << name << "': " << ex.what());
    }
  }
  return success;
}

bool LifecycleComponent::activate_outputs() {
  bool success = true;
  for (auto const& [name, interface]: this->outputs_) {
    try {
      interface->activate();
    } catch (const std::exception& ex) {
      success = false;
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to activate output '" << name << "': " << ex.what());
    }
  }
  return success;
}

bool LifecycleComponent::deactivate_outputs() {
  bool success = true;
  for (auto const& [name, interface]: this->outputs_) {
    try {
      interface->deactivate();
    } catch (const std::exception& ex) {
      success = false;
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to deactivate output '" << name << "': " << ex.what());
    }
  }
  return success;
}

}