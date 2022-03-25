#pragma once

#include "modulo_components/ComponentInterface.h"
#include "modulo_core/communication/EncodedState.hpp"

namespace modulo_components {

class Component : public ComponentInterface<rclcpp::Node, rclcpp::Publisher<modulo::core::EncodedState>> {

};

}