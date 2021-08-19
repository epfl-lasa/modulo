#pragma once

#include <functional>
#include <list>
#include <state_representation/parameters/Event.hpp>
#include <state_representation/parameters/Parameter.hpp>
#include <state_representation/parameters/ParameterInterface.hpp>
#include <state_representation/parameters/Predicate.hpp>

#include "modulo_core/Cell.hpp"

namespace modulo::core {
/**
 * @class Component
 * @brief Abstract class to define an component, handling base predicates
 */
class Component : public core::Cell {
private:
  std::map<std::string, std::shared_ptr<state_representation::Predicate>> predicates_;///< map of predicates
  std::list<std::pair<std::string, std::function<bool(void)>>> predicate_functions_;  ///< list of predicate functions evaluated at each step
  std::map<std::string, std::string> external_predicate_channels_;                    ///< map storing the channels for the external predicates

  /**
   * @brief Periodically called function that evaluates the predicates functions
   */
  void evaluate_predicate_functions();

protected:
  /**
   * @brief Add a predicate to the map of predicates
   * @param predicate the predicate to add
   */
  void add_predicate(const std::shared_ptr<state_representation::Predicate>& predicate);

  /**
   * @brief Add a predicate to the map of predicates based on a function to periodically call
   * @param predicate_name the name of the associated predicate
   * @param predicate_function the function to periodically call that returns the value of the predicate
   */
  void add_predicate(const std::string& predicate_name, const std::function<bool(void)>& predicate_function);

  /**
   * @brief Add an external predicate to the map of predicates which value will be registered through
   * the subscribed channel provided
   * @param predicate_name the name of the predicate
   * @param channel the associated subscribed channel
   */
  void add_received_predicate(const std::string& predicate_name, const std::string& channel);

  /**
   * @brief Get the value of the predicate given as parameter
   * @param predicate_name the name of the predicate to retrieve from the
   * map of predicates
   * @return the value of the predicate as a boolean
   */
  bool get_predicate_value(const std::string& predicate_name) const;

  /**
   * @brief Set the value of the predicate given as parameter
   * @param predicate_name the name of the predicate to retrieve from the
   * map of predicates
   * @param value the new value of the predicate
   */
  void set_predicate_value(const std::string& predicate_name, bool value);

  /**
   * @brief Add an event to the map of predicates
   * @param event the event to add
   */
  void add_event(const std::shared_ptr<state_representation::Event>& event);

  /**
   * @brief Add an event to the map of predicates based on a function to periodically call
   * @param event_name the name of the associated event
   * @param event_function the function to periodically call that returns the value of the event
   */
  void add_event(const std::string& event_name, const std::function<bool(void)>& event_function);

  /**
   * @brief Add an external event to the map of predicates which value will be registered through
   * the subscribed channel provided
   * @param event_name the name of the event
   * @param channel the associated subscribed channel
   */
  void add_received_event(const std::string& event_name, const std::string& channel);

  /**
   * @brief Read the value of the event given as parameter
   * @param event_name the name of the event to retrieve from the
   * map of predicates
   * @return the value of the evant as a boolean
   */
  bool read_event_value(const std::string& event_name);

  /**
   * @brief Set the value of the event given as parameter
   * @param event_name the name of the event to retrieve from the
   * map of predicates
   * @param value the new value of the event
   */
  void set_event_value(const std::string& event_name, bool value);

public:
  /**
   * @brief Constructor for the Component class
   * @tparam DurationT template value to accept any type durations from the std::chrono library
   * @param name name of the node
   * @param period rate used by each publisher of the class
   * @param intra_process_comms if true, activate intra processes communication by sharing pointers instead of messages
   */
  template <typename DurationT>
  explicit Component(const std::string& node_name,
                     const std::chrono::duration<int64_t, DurationT>& period,
                     bool intra_process_comms = false);

  /**
   * @brief Component construction from ROS2 NodeOptions
   * @param options NodeOptions containing a node name in the remapping arguments list and a "period" parameter
   * with a value in seconds in the parameter override list
   */
  explicit Component(const rclcpp::NodeOptions& options);

  /**
   * @brief Destructor
   */
  ~Component();

  /**
   * @brief Get the list of predicates of the action
   * @return the list of predicates
   */
  const std::list<std::shared_ptr<state_representation::Predicate>> get_predicates() const;

  /**
   * @brief Transition callback for state configuring
   * on_configure callback is being called when the lifecycle node
   * enters the "configuring" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "inactive" state or stays
   * in "unconfigured".
   * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
   * TRANSITION_CALLBACK_FAILURE transitions to "unconfigured"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Function computing one step of calculation. It is called periodically in the run function.
   */
  virtual void step() = 0;
};

template <typename DurationT>
Component::Component(const std::string& node_name,
                     const std::chrono::duration<int64_t, DurationT>& period,
                     bool intra_process_comms) : Cell(node_name, period, intra_process_comms) {
  this->declare_parameter<int>("predicate_checking_period", 100);
  this->add_predicate("is_configured", [this] { return this->is_configured(); });
  this->add_predicate("is_active", [this] { return this->is_active(); });
}
}// namespace modulo::core
