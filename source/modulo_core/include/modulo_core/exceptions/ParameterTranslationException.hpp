#pragma once

#include "modulo_core/exceptions/CoreException.hpp"

namespace modulo_core::exceptions {

/**
 * @class ParameterTranslationException
 * @brief An exception class to notify incompatibility when translating parameters from different sources.
 * @details This is an exception class to be thrown if there is a problem while translating from a ROS parameter to a
 * state_representation parameter and vice versa.
 */
class ParameterTranslationException : public CoreException {
public:
  explicit ParameterTranslationException(const std::string& msg) : CoreException(msg) {};
};
}// namespace modulo_core::exceptions


