#include "modulo_components/ComponentInterface.hpp"

namespace modulo_components {

template<class NodeT>
rclcpp::QoS ComponentInterface<NodeT>::get_qos() const {
  return this->qos_;
}

template<class NodeT>
void ComponentInterface<NodeT>::set_qos(const rclcpp::QoS& qos) {
  this->qos_ = qos;
}

}// namespace modulo_components
