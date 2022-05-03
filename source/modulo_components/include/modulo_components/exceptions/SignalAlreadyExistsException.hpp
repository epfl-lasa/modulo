#pragma once

#include <stdexcept>
#include <string>

namespace modulo_components::exceptions {
class SignalAlreadyExistsException : public std::runtime_error {
public:
  explicit SignalAlreadyExistsException(const std::string& msg) : std::runtime_error(msg) {};
};
}