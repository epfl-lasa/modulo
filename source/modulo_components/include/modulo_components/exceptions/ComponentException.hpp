#pragma once

#include <stdexcept>
#include <string>

/**
 * @namespace modulo_components::exceptions
 * @brief Modulo component exception classes.
 */
namespace modulo_components::exceptions {

/**
 * @class ComponentException
 * @brief A base class for all component exceptions.
 * @details This inherits from std::runtime_exception.
 */
class ComponentException : public std::runtime_error {
public:
  explicit ComponentException(const std::string& msg) : ComponentException("ComponentException", msg) {};
protected:
  ComponentException(const std::string& prefix, const std::string& msg) : std::runtime_error(prefix + ": " + msg) {}
};
}// namespace modulo_components::exceptions
