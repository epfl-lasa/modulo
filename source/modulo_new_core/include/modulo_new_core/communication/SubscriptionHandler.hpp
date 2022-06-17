#pragma once

#include "modulo_new_core/communication/SubscriptionInterface.hpp"
#include "modulo_new_core/exceptions/NullPointerException.hpp"

namespace modulo_new_core::communication {

/**
 * @class SubscriptionHandler
 * @brief The SubscriptionHandler handles different types of ROS subscriptions to receive data from those subscriptions.
 * @tparam MsgT The ROS message type of the ROS subscription
 */
template<typename MsgT>
class SubscriptionHandler : public SubscriptionInterface {
public:
  /**
   * @brief Constructor with the message pair.
   * @param message_pair The pointer to the message pair with the data that should be updated through the subscription
   */
  explicit SubscriptionHandler(std::shared_ptr<MessagePairInterface> message_pair = nullptr);

  /**
   * @brief Getter of the ROS subscription.
   */
  [[nodiscard]] std::shared_ptr<rclcpp::Subscription<MsgT>> get_subscription() const;

  /**
   * @brief Setter of the ROS subscription.
   * @throws NullPointerException if the provided subscription pointer is null
   */
  void set_subscription(const std::shared_ptr<rclcpp::Subscription<MsgT>>& subscription);

  /**
   * @brief Get a callback function that will be associated with the ROS subscription to receive and translate messages.
   */
  std::function<void(const std::shared_ptr<MsgT>)> get_callback();

  /**
   * @brief Create an SubscriptionInterface pointer through an instance of a SubscriptionHandler by providing a ROS
   * subscription.
   * @details This throws a NullPointerException if the ROS subscription is null.
   * @see SubscriptionHandler::set_subscription
   * @param subscription The ROS subscription
   * @return The resulting SubscriptionInterface pointer
   */
  std::shared_ptr<SubscriptionInterface>
  create_subscription_interface(const std::shared_ptr<rclcpp::Subscription<MsgT>>& subscription);

private:
  std::shared_ptr<rclcpp::Subscription<MsgT>> subscription_; ///< The pointer referring to the ROS subscription
};

template<typename MsgT>
std::shared_ptr<rclcpp::Subscription<MsgT>> SubscriptionHandler<MsgT>::get_subscription() const {
  return this->subscription_;
}

template<typename MsgT>
void SubscriptionHandler<MsgT>::set_subscription(const std::shared_ptr<rclcpp::Subscription<MsgT>>& subscription) {
  if (subscription == nullptr) {
    throw exceptions::NullPointerException("Provide a valid pointer");
  }
  this->subscription_ = subscription;
}

template<typename MsgT>
std::shared_ptr<SubscriptionInterface> SubscriptionHandler<MsgT>::create_subscription_interface(
    const std::shared_ptr<rclcpp::Subscription<MsgT>>& subscription
) {
  this->set_subscription(subscription);
  return std::shared_ptr<SubscriptionInterface>(this->shared_from_this());
}
}// namespace modulo_new_core::communication
