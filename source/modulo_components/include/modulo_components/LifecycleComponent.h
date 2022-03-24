#pragma once

#include "modulo_components/ComponentInterface.h"

namespace modulo_components {

class LifecycleComponent : public ComponentInterface<rclcpp_lifecycle::LifecycleNode, rclcpp_lifecycle::LifecyclePublisher<modulo::core::EncodedState>> {

};

}