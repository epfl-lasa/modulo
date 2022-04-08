#pragma once

#include <stdexcept>
#include <string>

namespace modulo_new_core::exceptions {
class InvalidPointerCastException : public std::runtime_error {
public:
  explicit InvalidPointerCastException(const std::string& msg) : std::runtime_error(msg) {};
};
}