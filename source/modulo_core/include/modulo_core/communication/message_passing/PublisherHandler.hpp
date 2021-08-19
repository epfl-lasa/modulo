#pragma once

#include <rclcpp/publisher.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <tuple>

#include "modulo_core/communication/message_passing/MessagePassingHandler.hpp"

namespace modulo::core::communication {
/**
 * @class PublisherHandler
 * @brief Class to define a publisher
 * @tparam RecT the type of recipient (of state_representation::State base class)
 * @tparam MsgT the type of associated ROS2 message
 *
 */
template <class RecT, typename MsgT>
class PublisherHandler : public MessagePassingHandler {
private:
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MsgT>> publisher_;///< reference to the ROS2 publisher
  std::shared_ptr<rclcpp::Clock> clock_;                                 ///< reference to the Cell clock
  std::shared_ptr<rclcpp::TimerBase> timer_;                             ///< reference to the associated timer
  bool activated_;                                                       ///< indicate if the publisher is activated or not

protected:
  /**
   * @brief Function to publish the values of the recipient over the network
   * @param recipient [description]
   */
  void publish(const RecT& recipient);

public:
  /**
   * @brief Constructor of a PublisherHandler
   * @param  recipient the recipent associated to the publisher
   * @param  clock     reference to the Cell clock
   */
  explicit PublisherHandler(const std::shared_ptr<state_representation::State>& recipient,
                            const std::shared_ptr<rclcpp::Clock>& clock);

  /**
   * @brief Constructor of a PublisherHandler without a recipient for one-shot publishing
   * @param  clock     reference to the Cell clock
   */
  explicit PublisherHandler(const std::shared_ptr<rclcpp::Clock>& clock);

  /**
   * @brief Function to publish periodically 
   */
  void publish_callback();

  /**
   * @brief Getter of the clock reference
   * @return the clock reference
   */
  const rclcpp::Clock& get_clock() const;

  /**
   * @brief Getter of the clock as a non const reference
   * @return the clock reference
   */
  rclcpp::Clock& get_clock();

  /**
   * @brief Setter of the publisher reference
   * @param publisher the reference to the publisher
   */
  void set_publisher(const std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MsgT>>& publisher);

  /**
   * @brief Setter of the timer reference
   * @param timer the reference to the timer
   */
  void set_timer(const std::shared_ptr<rclcpp::TimerBase>& timer);

  /**
   * @brief Function to activate the publisher
   */
  void activate();

  /**
   * @brief Function to deactivate the publisher
   */
  void deactivate();
};

template <class RecT, typename MsgT>
PublisherHandler<RecT, MsgT>::PublisherHandler(const std::shared_ptr<state_representation::State>& recipient,
                                               const std::shared_ptr<rclcpp::Clock>& clock) : MessagePassingHandler(CommunicationType::PUBLISHER,
                                                                                                                    recipient),
                                                                                              clock_(clock),
                                                                                              activated_(false) {}

template <class RecT, typename MsgT>
PublisherHandler<RecT, MsgT>::PublisherHandler(const std::shared_ptr<rclcpp::Clock>& clock) : MessagePassingHandler(CommunicationType::PUBLISHER),
                                                                                              clock_(clock),
                                                                                              activated_(false) {}

template <class RecT, typename MsgT>
void PublisherHandler<RecT, MsgT>::publish(const RecT& recipient) {
  auto out_msg = std::make_unique<MsgT>();
  state_conversion::write_msg(*out_msg, recipient, this->get_clock().now());
  this->publisher_->publish(std::move(out_msg));
}

template <class RecT, typename MsgT>
void PublisherHandler<RecT, MsgT>::publish_callback() {
  if (this->is_asynchronous() && this->activated_ && !this->get_recipient().is_empty()) {
    this->publish(static_cast<RecT&>(this->get_recipient()));
  }
}

template <class RecT, typename MsgT>
inline const rclcpp::Clock& PublisherHandler<RecT, MsgT>::get_clock() const {
  return *this->clock_;
}

template <class RecT, typename MsgT>
inline rclcpp::Clock& PublisherHandler<RecT, MsgT>::get_clock() {
  return *this->clock_;
}

template <class RecT, typename MsgT>
inline void PublisherHandler<RecT, MsgT>::set_publisher(const std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MsgT>>& publisher) {
  this->publisher_ = std::move(publisher);
}

template <class RecT, typename MsgT>
inline void PublisherHandler<RecT, MsgT>::set_timer(const std::shared_ptr<rclcpp::TimerBase>& timer) {
  this->timer_ = std::move(timer);
}

template <class RecT, typename MsgT>
inline void PublisherHandler<RecT, MsgT>::activate() {
  this->activated_ = true;
  this->publisher_->on_activate();
}

template <class RecT, typename MsgT>
inline void PublisherHandler<RecT, MsgT>::deactivate() {
  this->activated_ = false;
  this->publisher_->on_deactivate();
}
}// namespace modulo::core::communication