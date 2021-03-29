#include "modulo_core/Recorder.hpp"

namespace modulo::core {
Recorder::~Recorder() {
  this->on_shutdown();
}

bool Recorder::on_configure() {
  return true;
}

bool Recorder::on_activate() {
  this->start_time = std::chrono::system_clock::now();
  return true;
}

bool Recorder::on_deactivate() {
  this->end_time = std::chrono::system_clock::now();
  return true;
}

bool Recorder::on_cleanup() {
  return true;
}

bool Recorder::on_shutdown() {
  return true;
}

void Recorder::step() {
  for (auto& h : this->get_handlers()) {
    if (h.second.first->get_type() == communication::CommunicationType::SUBSCRIPTION) {
      const communication::MessagePassingHandler& subscription = static_cast<const communication::MessagePassingHandler&>(*h.second.first);
      if (!this->record(subscription.get_recipient())) RCLCPP_ERROR(this->get_logger(), "Unable to record " + subscription.get_recipient().get_name());
    }
  }
}

bool Recorder::record(const state_representation::State& state) const {
  switch (state.get_type()) {
    case state_representation::StateType::CARTESIANSTATE:
      return record(static_cast<const state_representation::CartesianState&>(state));

    case state_representation::StateType::JOINTSTATE:
      return record(static_cast<const state_representation::JointState&>(state));

    default:
      RCLCPP_ERROR(this->get_logger(), "Recording function for " + state.get_name() + " not defined for this type of state");
      return false;
  }
}

bool Recorder::record(const state_representation::CartesianState& state) const {
  RCLCPP_WARN(this->get_logger(), "Trying to record " + state.get_name() + " from the base class");
  return false;
}

bool Recorder::record(const state_representation::JointState& state) const {
  RCLCPP_WARN(this->get_logger(), "Trying to record " + state.get_name() + " from the base class");
  return false;
}
}// namespace modulo::core
