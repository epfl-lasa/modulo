#pragma once

#include <state_representation/parameters/ParameterMap.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "modulo_core/communication/EncodedState.hpp"

namespace modulo_components {

template<class T, typename PubT>
class ComponentInterface : public state_representation::ParameterMap, public T {

public:
  explicit ComponentInterface(const rclcpp::NodeOptions& node_options);

};


}// namespace modulo_components