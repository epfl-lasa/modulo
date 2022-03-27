#pragma once

#include "modulo_components/ComponentInterface.h"
#include "modulo_new_core/EncodedState.hpp"

namespace modulo_components {

class LifecycleComponent : public ComponentInterface<rclcpp_lifecycle::LifecycleNode,
                                                     rclcpp_lifecycle::LifecyclePublisher<modulo_new_core::EncodedState>> {

};

}