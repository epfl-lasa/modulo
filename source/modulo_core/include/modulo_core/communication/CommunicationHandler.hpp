#pragma once

#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace modulo::core::communication {
/**
 * @enum CommunicationType
 * @brief enumeration of the possible type of communication interfaces
 */
enum class CommunicationType {
  PUBLISHER,
  SUBSCRIPTION,
  TRANSFORMLISTENER,
  CLIENT,
};

/**
 * @class CommunicationHandler
 * @brief Abstract class to define a Communication interface
 */
class CommunicationHandler {
private:
  const CommunicationType type_;    ///< type of the handler from the CommunicationType enumeration
  std::chrono::nanoseconds timeout_;///< period before considered time out

public:
  /**
   * @brief Constructor for a CommunicationHandler
   * @param  type      the type of CommunicationHandler from the CommunicationType enumeration
   * @param  clock     reference to the Cell clock
   */
  explicit CommunicationHandler(const CommunicationType& type);

  /**
	 * @brief Constructor for a CommunicationHandler  with a timeout
	 * @param  type      the type of CommunicationHandler from the CommunicationType enumeration
	 * @param  timeout   period before considered time out
	 * @param  clock     reference to the Cell clock
	 */
  template <typename DurationT>
  explicit CommunicationHandler(const CommunicationType& type,
                                const std::chrono::duration<int64_t, DurationT>& timeout);

  /**
   * @brief Getter of the CommunicationType
   * @return the type of the handler
   */
  const CommunicationType& get_type();

  /**
   * @brief Getter of the timeout period
   * @return the timeout period
   */
  const std::chrono::nanoseconds& get_timeout() const;

  /**
   * @brief Virtual function to activate a handler
   */
  virtual void activate();

  /**
   * @brief Virtual function to deactivate a handler
   */
  virtual void deactivate();

  /**
   * @brief Virtual function to check if the handler is timed out
   */
  virtual void check_timeout();
};

template <typename DurationT>
CommunicationHandler::CommunicationHandler(const CommunicationType& type,
                                           const std::chrono::duration<int64_t, DurationT>& timeout) : type_(type),
                                                                                                       timeout_(timeout) {}

inline const CommunicationType& CommunicationHandler::get_type() {
  return this->type_;
}

inline const std::chrono::nanoseconds& CommunicationHandler::get_timeout() const {
  return this->timeout_;
}

inline void CommunicationHandler::activate() {}

inline void CommunicationHandler::deactivate() {}

inline void CommunicationHandler::check_timeout() {}
}// namespace modulo::core::communication
