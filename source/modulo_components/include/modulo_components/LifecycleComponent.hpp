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
   * @brief Constructor from node options
   * @param node_options node options as used in ROS2 Node
   */
  explicit LifecycleComponent(const rclcpp::NodeOptions& node_options);

protected:
  template<typename DataT>
  void add_output(const std::string& signal_name, const std::shared_ptr<DataT>& data, bool fixed_topic = false);

private:
  void configure_outputs();

  void activate_outputs();

};

template<typename DataT>
void
LifecycleComponent::add_output(const std::string& signal_name, const std::shared_ptr<DataT>& data, bool fixed_topic) {
  // TODO parse signal name
  this->create_output(signal_name, data, fixed_topic);
}

}