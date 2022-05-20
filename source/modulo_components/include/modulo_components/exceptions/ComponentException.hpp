#pragma once

#include <stdexcept>
#include <string>

namespace modulo_components::exceptions {
class ComponentException : public std::runtime_error {
public:
  explicit ComponentException(const std::string& msg) : std::runtime_error(msg) {};
};
}