#pragma once

#include <stdexcept>
#include <string>

namespace modulo_components::exceptions {

/**
 * @class ComponentException
 * @brief A base class for all component exceptions.
 * @details This inherits from std::runtime_exception.
 */
class ComponentException : public std::runtime_error {
public:
  explicit ComponentException(const std::string& msg) : std::runtime_error(msg) {};
};

}// namespace modulo_components::exceptions