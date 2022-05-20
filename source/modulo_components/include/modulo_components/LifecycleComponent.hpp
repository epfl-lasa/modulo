#pragma once

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "modulo_components/ComponentInterface.hpp"

#include "modulo_new_core/EncodedState.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

namespace modulo_components {

class LifecycleComponent : public ComponentInterface<rclcpp_lifecycle::LifecycleNode> {
public:
  friend class LifecycleComponentPublicInterface;

  /**
   * @brief Constructor from node options.
   * @param node_options Node options as used in ROS2 LifecycleNode
   */
  explicit LifecycleComponent(const rclcpp::NodeOptions& node_options);

  /**
   * @brief Virtual default destructor.
   */
  virtual ~LifecycleComponent() = default;

protected:
  /**
   * @brief Steps to execute when configuring the component.
   */
  bool on_configure();

  /**
   * @brief Steps to execute when cleaning up the component.
   */
  bool on_cleanup();

  /**
   * @brief Steps to execute when activating the component.
   */
  bool on_activate();

  /**
   * @brief Steps to execute when deactivating the component.
   */
  bool on_deactivate();

  /**
   * @brief Steps to execute when shutting down the component.
   */
  bool on_shutdown();
  /**
   * @brief Steps to execute periodically. To be redefined by derived Component classes.
   */
  virtual void on_step();

  /**
   * @brief Add an output signal of the component.
   * @tparam DataT Type of the data pointer
   * @param signal_name Name of the output signal
   * @param data Data to transmit on the output signal
   * @param fixed_topic If true, the topic name of the output signal is fixed
   * @param default_topic If set, the default value for the topic name to use
   */
  template<typename DataT>
  void add_output(const std::string& signal_name, const std::shared_ptr<DataT>& data, bool fixed_topic = false);

private:
  /**
   * @brief Transition callback for state 'Configuring'.
   * @details on_configure callback is being called when the lifecycle node enters the 'Configuring' state.
   * Depending on the return value of this function, the node may either transition to the 'Inactive' state via the
   * 'configure' transition or stays 'Unconfigured.\n
   * TRANSITION_CALLBACK_SUCCESS transitions to 'Inactive'\n
   * TRANSITION_CALLBACK_FAILURE transitions to 'Unconfigured'\n
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to 'ErrorProcessing'\n
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State&) override;

  /**
    * @brief Transition callback for state 'CleaningUp'.
    * @details on_cleanup callback is being called when the lifecycle node enters the 'CleaningUp' state.
    * Depending on the return value of this function, the node may either transition to the 'Unconfigured' state via the
    * 'cleanup' transition or goes to 'ErrorProcessing'.\n
    * TRANSITION_CALLBACK_SUCCESS transitions to 'Unconfigured'\n
    * TRANSITION_CALLBACK_FAILURE, TRANSITION_CALLBACK_ERROR or any uncaught exceptions to 'ErrorProcessing'\n
    */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State&) override;

  /**
  * @brief Transition callback for state 'Activating'.
  * @details on_activate callback is being called when the lifecycle node enters the 'Activating' state.
  * Depending on the return value of this function, the node may either transition to the 'Active' state via the
  * 'activate' transition or goes to 'ErrorProcessing'.\n
  * TRANSITION_CALLBACK_SUCCESS transitions to 'Active'\n
  * TRANSITION_CALLBACK_FAILURE, TRANSITION_CALLBACK_ERROR or any uncaught exceptions to 'ErrorProcessing'\n
  */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State&) override;

  /**
    * @brief Transition callback for state 'Deactivating'.
    * @details on_deactivate callback is being called when the lifecycle node enters the 'Deactivating' state.
    * Depending on the return value of this function, the node may either transition to the 'Inactive' state via the
    * 'deactivate' transition or goes to 'ErrorProcessing'.\n
    * TRANSITION_CALLBACK_SUCCESS transitions to 'Inactive'\n
    * TRANSITION_CALLBACK_FAILURE, TRANSITION_CALLBACK_ERROR or any uncaught exceptions to 'ErrorProcessing'\n
    */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State&) override;

  /**
    * @brief Transition callback for state 'Deactivating'.
    * @details on_deactivate callback is being called when the lifecycle node enters the 'Deactivating' state.
    * Depending on the return value of this function, the node may either transition to the 'Inactive' state via the
    * 'deactivate' transition or goes to 'ErrorProcessing'.\n
    * TRANSITION_CALLBACK_SUCCESS transitions to "Inactive"\n
    * TRANSITION_CALLBACK_FAILURE, TRANSITION_CALLBACK_ERROR or any uncaught exceptions to 'ErrorProcessing'\n
    */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Step function that is called periodically and publishes predicates,
   * outputs, evaluates daemon callbacks, and calls the on_step function.
   */
  void step() override;

  /**
   * @brief Configure all outputs.
   * @return True if configuration was successful
   */
  bool configure_outputs();

  /**
   * @brief Activate all outputs.
   * @return True if activation was successful
   */
  bool activate_outputs();

  /**
   * @brief Deactivate all outputs.
   * @return True if deactivation was successful
   */
  bool deactivate_outputs();

  using rclcpp_lifecycle::LifecycleNode::create_publisher;
  using ComponentInterface<rclcpp_lifecycle::LifecycleNode>::create_output;
  using ComponentInterface<rclcpp_lifecycle::LifecycleNode>::outputs_;
  using ComponentInterface<rclcpp_lifecycle::LifecycleNode>::qos_;
  using ComponentInterface<rclcpp_lifecycle::LifecycleNode>::publish_predicates;
  using ComponentInterface<rclcpp_lifecycle::LifecycleNode>::publish_outputs;
  using ComponentInterface<rclcpp_lifecycle::LifecycleNode>::evaluate_periodic_callbacks;
};

inline void LifecycleComponent::on_step() {}

template<typename DataT>
inline void
LifecycleComponent::add_output(const std::string& signal_name, const std::shared_ptr<DataT>& data, bool fixed_topic) {
  try {
    std::string parsed_signal_name = utilities::parse_signal_name(signal_name);
    this->create_output(parsed_signal_name, data, fixed_topic);
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to add output '" << signal_name << "': " << ex.what());
  }
}

// TODO, if we raise error we need to manually call the lifecycle change state callback,
// call callback function that this service calls

} //namespace modulo_components