#pragma once

#include <state_representation/exceptions/InvalidParameterException.hpp>

namespace modulo_new_core::exceptions {

/**
 * @class IncompatibleParameterException
 * @brief An exception class to notify incompatibility when translating parameters from different sources.
 * @details This is an exception class to be thrown if there is a problem while translating from a ROS parameter to a
 * state_representation parameter and vice versa.
 */
class IncompatibleParameterException : public state_representation::exceptions::InvalidParameterException {
public:
  explicit IncompatibleParameterException(const std::string& msg) :
      state_representation::exceptions::InvalidParameterException(msg) {};
};
}// namespace modulo_new_core::exceptions
