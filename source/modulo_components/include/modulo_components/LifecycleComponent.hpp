#pragma once

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "modulo_components/ComponentInterface.hpp"

#include "modulo_new_core/EncodedState.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

namespace modulo_components {

class LifecycleComponent : public ComponentInterface<rclcpp_lifecycle::LifecycleNode> {

};

}