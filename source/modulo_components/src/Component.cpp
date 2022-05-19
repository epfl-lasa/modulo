#include "modulo_components/Component.hpp"

using namespace modulo_new_core::communication;

namespace modulo_components {

Component::Component(const rclcpp::NodeOptions& node_options, bool start_thread) :
    ComponentInterface<rclcpp::Node>(node_options, PublisherType::PUBLISHER), started_(false) {
  this->add_predicate("in_error_state", false);
  this->add_predicate("is_finished", false);

  if (start_thread) {
    this->start_thread();
  }
}

void Component::step() {
  this->publish_predicates();
  this->publish_outputs();
  this->evaluate_periodic_callbacks();
}

void Component::start_thread() {
  if (this->started_) {
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Run thread for component '" << this->get_name() << "has already been started");
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
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Failed to run component '" << this->get_name() << "': " << ex.what());
    this->raise_error();
    return;
  }
  this->set_predicate("is_finished", true);
}

bool Component::execute() {
  return true;
}

void Component::raise_error() {
  this->set_predicate("in_error_state", true);
}

}// namespace modulo_components