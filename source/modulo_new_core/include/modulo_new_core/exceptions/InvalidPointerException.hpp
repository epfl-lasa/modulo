#pragma once

#include <stdexcept>
#include <string>

namespace modulo_new_core::exceptions {
class InvalidPointerException : public std::runtime_error {
public:
  explicit InvalidPointerException(const std::string& msg) : std::runtime_error(msg) {};
};
}