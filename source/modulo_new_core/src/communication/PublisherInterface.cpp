#include "modulo_new_core/communication/PublisherInterface.hpp"

#include <rclcpp/publisher.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include "modulo_new_core/communication/PublisherHandler.hpp"
#include "modulo_new_core/exceptions/NullPointerException.hpp"

namespace modulo_new_core::communication {

PublisherInterface::PublisherInterface(PublisherType type) : type_(type) {}

PublisherType PublisherInterface::get_type() const {
  return this->type_;
}

}// namespace modulo_new_core::communication