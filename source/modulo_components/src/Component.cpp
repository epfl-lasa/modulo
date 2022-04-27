#include "modulo_components/Component.hpp"

using namespace modulo_new_core::communication;

namespace modulo_components {

Component::Component(const rclcpp::NodeOptions& node_options) :
    ComponentInterface<rclcpp::Node>(node_options, PublisherType::PUBLISHER) {}

}