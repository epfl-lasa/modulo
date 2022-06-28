#include "modulo_components/Component.hpp"

using namespace modulo_new_core::communication;

namespace modulo_components {

Component::Component(const rclcpp::NodeOptions& node_options, bool start_thread, const std::string& fallback_name) :
    ComponentInterface<rclcpp::Node>(node_options, PublisherType::PUBLISHER, fallback_name), started_(false) {
  this->add_predicate("is_finished", false);

  if (start_thread) {
    this->start_thread();
  }
}

void Component::step() {
  try {
    this->publish_predicates();
    this->publish_outputs();
    this->evaluate_periodic_callbacks();
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to execute step function:" << ex.what());
    this->raise_error();
  }
}

void Component::start_thread() {
  if (this->started_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to start run thread: Thread has already been started.");
    return;
  }
  this->started_ = true;
  this->run_thread_ = std::thread([this]() { this->run(); });
}

void Component::run() {
  try {
    if (!this->execute()) {
      this->raise_error();
      return;
    }
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to run the execute function: " << ex.what());
    this->raise_error();
    return;
  }
  RCLCPP_DEBUG(this->get_logger(), "Execution finished, setting is_finished predicate to true");
  this->set_predicate("is_finished", true);
}

bool Component::execute() {
  return true;
}
}// namespace modulo_components
