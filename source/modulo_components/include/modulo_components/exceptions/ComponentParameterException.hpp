#pragma once

#include "modulo_components/exceptions/ComponentException.hpp"

namespace modulo_components::exceptions {

/**
 * @class ComponentParameterException
 * @brief An exception class to notify errors with component parameters.
 * @details This is an exception class to be thrown if there is a problem with component parameters
 * (overriding, inconsistent types, undeclared, ...).
 */
class ComponentParameterException : public ComponentException {
public:
  explicit ComponentParameterException(const std::string& msg) : ComponentException(msg) {};
};
}// namespace modulo_components::exceptions
