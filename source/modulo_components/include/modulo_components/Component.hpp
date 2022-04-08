#pragma once

#include <rclcpp/node.hpp>

#include "modulo_components/ComponentInterface.hpp"
#include "modulo_new_core/EncodedState.hpp"

namespace modulo_components {

class Component : public ComponentInterface<rclcpp::Node> {

};

}