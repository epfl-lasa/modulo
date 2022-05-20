#include "modulo_components/LifecycleComponent.hpp"

#include <lifecycle_msgs/msg/state.hpp>

using namespace modulo_new_core::communication;

namespace modulo_components {

LifecycleComponent::LifecycleComponent(const rclcpp::NodeOptions& node_options) :
    ComponentInterface<rclcpp_lifecycle::LifecycleNode>(node_options, PublisherType::LIFECYCLE_PUBLISHER) {
  this->add_predicate("is_unconfigured", true);
  this->add_predicate("is_inactive", false);
  this->add_predicate("is_active", false);
  this->add_predicate("is_shutdown", false);
}

void LifecycleComponent::step() {
  if (this->get_predicate("is_active")) {
    this->publish_predicates();
    this->publish_outputs();
    this->evaluate_daemon_callbacks();
    this->on_step();
  }
}

bool LifecycleComponent::on_configure() {
  return this->configure_outputs();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleComponent::on_configure(const rclcpp_lifecycle::State&) {
  RCLCPP_DEBUG(this->get_logger(), "on_configure is called.");
  if (!this->on_configure()) {
    RCLCPP_ERROR(get_logger(), "Failed to configure component.");
    // TODO need reset?
//    this->reset();
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  this->set_predicate("is_unconfigured", false);
  this->set_predicate("is_inactive", true);
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool LifecycleComponent::on_cleanup() {
  return true;
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleComponent::on_cleanup(const rclcpp_lifecycle::State&) {
  RCLCPP_DEBUG(this->get_logger(), "on_cleanup is called.");
  if (!this->on_cleanup()) {
    RCLCPP_ERROR(get_logger(), "Failed to clean up component.");
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  // TODO
//  this->reset();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool LifecycleComponent::on_activate() {
  return this->activate_outputs();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleComponent::on_activate(const rclcpp_lifecycle::State&) {
  RCLCPP_DEBUG(this->get_logger(), "on_activate is called.");
  if (!this->on_activate()) {
    RCLCPP_ERROR(get_logger(), "Failed to activate component.");
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  this->set_predicate("is_inactive", false);
  this->set_predicate("is_active", true);
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool LifecycleComponent::on_deactivate() {
  return this->deactivate_outputs();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleComponent::on_deactivate(const rclcpp_lifecycle::State&) {
  RCLCPP_DEBUG(this->get_logger(), "on_deactivate is called.");
  if (!this->on_deactivate()) {
    RCLCPP_ERROR(get_logger(), "Failed to deactivate component.");
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  this->set_predicate("is_active", false);
  this->set_predicate("is_inactive", true);
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool LifecycleComponent::on_shutdown() {
  return true;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleComponent::on_shutdown(const rclcpp_lifecycle::State& state) {
  RCLCPP_DEBUG_STREAM(this->get_logger(), "on_shutdown is called from state '" << state.label() << "'.");
  auto current_state = state.id();
  // if the node is already shutdown just return success
  if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED) {
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
  // check current state and eventually deactivate and unconfigure
  if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_DEBUG(this->get_logger(), "Component is active, deactivating it before shutdown.");
    auto callback_return = this->on_deactivate(this->get_current_state());
    if (callback_return != LifecycleNodeInterface::CallbackReturn::SUCCESS) {
      return callback_return;
    }
    current_state = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
  }
  if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    RCLCPP_DEBUG(this->get_logger(), "Component is active, cleaning it up before shutdown.");
    auto callback_return = this->on_cleanup(this->get_current_state());
    if (callback_return != LifecycleNodeInterface::CallbackReturn::SUCCESS) {
      return callback_return;
    }
    current_state = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
  }
  if (!this->on_shutdown()) {
    RCLCPP_ERROR(get_logger(), "Failed to shut down component.");
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  // TODO reset
//  this->reset();
//  this->handlers_.clear();
//  this->daemons_.clear();
//  this->parameters_.clear();
//  this->shutdown_ = true;
  this->set_predicate("is_unconfigured", false);
  this->set_predicate("is_inactive", false);
  this->set_predicate("is_active", false);
  // TODO where do we set shutdown to true otherwise?
  this->set_predicate("is_shutdown", true);
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
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

}// namespace modulo_components