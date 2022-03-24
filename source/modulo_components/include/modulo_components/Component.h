#pragma once

#include "modulo_components/ComponentInterface.h"

namespace modulo_components {

class Component : public ComponentInterface<rclcpp::Node, rclcpp::Publisher<modulo::core::EncodedState>> {

};

}