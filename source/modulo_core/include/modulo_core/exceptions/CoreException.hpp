#pragma once

#include <stdexcept>
#include <string>

namespace modulo_core::exceptions {

/**
 * @class CoreException
 * @brief A base class for all core exceptions.
 * @details This inherits from std::runtime_exception.
 */
class CoreException : public std::runtime_error {
public:
  explicit CoreException(const std::string& msg) : std::runtime_error(msg) {};
};
}// namespace modulo_core::exceptions
