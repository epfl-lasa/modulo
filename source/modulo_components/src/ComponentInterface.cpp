#include "modulo_components/ComponentInterface.hpp"

namespace modulo_components {

template<NodeT>
rclcpp::QoS ComponentInterface<NodeT>::get_qos() const {
  return this->qos_;
}

template<NodeT>
void ComponentInterface<NodeT>::set_qos(const rclcpp::QoS& qos) {
  this->qos_ = qos;
}

}// namespace modulo_components
