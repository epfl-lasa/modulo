#pragma once

#include <state_representation/parameters/ParameterMap.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace modulo_components {

template<class NodeT, typename PubT>
class ComponentInterface : public state_representation::ParameterMap, public NodeT {

public:
  explicit ComponentInterface(const rclcpp::NodeOptions& node_options);

};

}// namespace modulo_components