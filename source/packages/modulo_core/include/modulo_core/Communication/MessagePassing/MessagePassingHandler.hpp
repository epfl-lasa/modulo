#pragma once

#include "modulo_core/Communication/CommunicationHandler.hpp"
#include "modulo_core/Communication/MessagePassing/ReadStateConversion.hpp"
#include "modulo_core/Communication/MessagePassing/WriteStateConversion.hpp"
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace modulo::core::communication {
/**
 * @class MessagePassingHandler
 * @brief Abstract class to define a MessagePassingHandler type communication interface
 */
class MessagePassingHandler : public CommunicationHandler {
private:
  std::shared_ptr<StateRepresentation::State> recipient_ = nullptr;///< recipient associated to the handler
  bool asynchronous_;                                              ///< indicate if the handler is asynchronous or not (for publishing mainly)

public:
  /**
   * @brief Constructor for a MessagePassingHandler without timeout nor recipient
   * @param  type      the type of MessagePassingHandler from the CommunicationType enumeration
   * @param  mutex     reference to the Cell mutex
   */
  explicit MessagePassingHandler(const CommunicationType& type,
                                 const std::shared_ptr<std::mutex>& mutex);

  /**
   * @brief Constructor for a MessagePassingHandler without timeout
   * @param  type      the type of MessagePassingHandler from the CommunicationType enumeration
   * @param  recipient recipient associated to the handler
   * @param  mutex     reference to the Cell mutex
   */
  explicit MessagePassingHandler(const CommunicationType& type,
                                 const std::shared_ptr<StateRepresentation::State>& recipient,
                                 const std::shared_ptr<std::mutex>& mutex);

  /**
   * @brief Constructor for a MessagePassingHandler
   * @param  type      the type of CommunicationHandler from the CommunicationType enumeration
   * @param  recipient recipient associated to the handler
   * @param  timeout   period before considered time out
   * @param  mutex     reference to the Cell mutex
   */
  template <typename DurationT>
  explicit MessagePassingHandler(const CommunicationType& type,
                                 const std::shared_ptr<StateRepresentation::State>& recipient,
                                 const std::chrono::duration<int64_t, DurationT>& timeout,
                                 const std::shared_ptr<std::mutex>& mutex);

  /**
   * @brief Getter of the recipient
   * @return the recipient
   */
  const StateRepresentation::State& get_recipient() const;

  /**
   * @brief Getter of the recipient as a non const value
   * @return the recipient
   */
  StateRepresentation::State& get_recipient();

  /**
   * @brief Getter of the asynchronous attribute
   * @return bool true if asynchronous
   */
  bool is_asynchronous() const;
};

template <typename DurationT>
MessagePassingHandler::MessagePassingHandler(const CommunicationType& type,
                                             const std::shared_ptr<StateRepresentation::State>& recipient,
                                             const std::chrono::duration<int64_t, DurationT>& timeout,
                                             const std::shared_ptr<std::mutex>& mutex) : CommunicationHandler(type, timeout, mutex),
                                                                                         recipient_(recipient),
                                                                                         asynchronous_(true) {}

inline const StateRepresentation::State& MessagePassingHandler::get_recipient() const {
  return *this->recipient_;
}

inline StateRepresentation::State& MessagePassingHandler::get_recipient() {
  return *this->recipient_;
}

inline bool MessagePassingHandler::is_asynchronous() const {
  return this->asynchronous_;
}
}// namespace modulo::core::communication
