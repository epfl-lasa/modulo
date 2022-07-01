#pragma once

#include "modulo_core/exceptions/CoreException.hpp"

namespace modulo_core::exceptions {

/**
 * @class IncompatibleParameterException
 * @brief An exception class to notify incompatibility when translating parameters from different sources.
 * @details This is an exception class to be thrown if there is a problem while translating from a ROS parameter to a
 * state_representation parameter and vice versa.
 */
class IncompatibleParameterException : public CoreException {
public:
  explicit IncompatibleParameterException(const std::string& msg) : CoreException(msg) {};
};
}// namespace modulo_core::exceptions


