#include "modulo_core/Component.hpp"

#include "modulo_core/exceptions/PredicateAlreadyRegisteredException.hpp"
#include "modulo_core/exceptions/PredicateNotFoundException.hpp"

namespace modulo::core {

Component::Component(const rclcpp::NodeOptions& options) : Cell(options) {
  this->declare_parameter<int>("predicate_checking_period", 100);
  this->add_predicate("is_configured", [this] { return this->is_configured(); });
  this->add_predicate("is_active", [this] { return this->is_active(); });
}

Component::~Component() {
  this->on_shutdown();
}

void Component::evaluate_predicate_functions() {
  for (auto const& [key, val] : this->predicate_functions_) {
    this->set_predicate_value(key, (val)());
  }
}

void Component::add_predicate(const std::shared_ptr<state_representation::Predicate>& predicate) {
  std::string predicate_name = predicate->get_name();
  if (this->predicates_.find(predicate_name) != this->predicates_.end()) {
    throw exceptions::PredicateAlreadyRegisteredException(predicate_name);
  }
  // add the predicate to the map
  this->predicates_.insert(std::make_pair(predicate_name, predicate));
  // add the publisher predicate
  this->add_publisher<std_msgs::msg::Bool>("/predicates/" + std::string(this->get_name()) + "/" + predicate_name, predicate, true);
}

void Component::add_predicate(const std::string& predicate_name, const std::function<bool(void)>& predicate_function) {
  // crate a predicate with named prefixed with the action name
  auto predicate = std::make_shared<state_representation::Predicate>(predicate_name);
  this->add_predicate(predicate);
  // insert the predicate function in the list
  this->predicate_functions_.insert(std::make_pair(predicate_name, predicate_function));
}

void Component::add_received_predicate(const std::string& predicate_name, const std::string& channel) {
  auto predicate = std::make_shared<state_representation::Predicate>(predicate_name);
  this->predicates_.insert(std::make_pair(predicate_name, predicate));
  // add a subscription to the channel where the predicate is published
  this->add_subscription<std_msgs::msg::Bool>(channel, predicate, true);
}

bool Component::get_predicate_value(const std::string& predicate_name) const {
  auto predicate_iterator = this->predicates_.find(predicate_name);
  if (predicate_iterator == this->predicates_.end()) {
    throw exceptions::PredicateNotFoundException(predicate_name);
  }
  return predicate_iterator->second->get_value();
}

void Component::set_predicate_value(const std::string& predicate_name, bool value) {
  auto predicate_iterator = this->predicates_.find(predicate_name);
  if (predicate_iterator == this->predicates_.end()) {
    throw exceptions::PredicateNotFoundException(predicate_name);
  }
  return predicate_iterator->second->set_value(value);
}

void Component::add_event(const std::shared_ptr<state_representation::Event>& event) {
  this->add_predicate(event);
}

void Component::add_event(const std::string& event_name, const std::function<bool(void)>& event_function) {
  this->add_predicate(event_name, event_function);
}

void Component::add_received_event(const std::string& event_name, const std::string& channel) {
  this->add_received_predicate(event_name, channel);
}

bool Component::read_event_value(const std::string& event_name) {
  auto event_iterator = this->predicates_.find(event_name);
  if (event_iterator == this->predicates_.end()) {
    throw exceptions::PredicateNotFoundException(event_name);
  }
  return std::dynamic_pointer_cast<state_representation::Event>(event_iterator->second)->read_value();
}

void Component::set_event_value(const std::string& event_name, bool value) {
  this->set_predicate_value(event_name, value);
}

const std::list<std::shared_ptr<state_representation::Predicate>> Component::get_predicates() const {
  std::list<std::shared_ptr<state_representation::Predicate>> predicate_list;
  std::transform(this->predicates_.begin(), this->predicates_.end(), std::back_inserter(predicate_list), [](const std::map<std::string, std::shared_ptr<state_representation::Predicate>>::value_type& val) { return val.second; });
  return predicate_list;
}
}// namespace modulo::core
