#pragma once

#include <stdexcept>
#include <string>

namespace modulo_new_core::exceptions {
class InvalidMessagePairCastException : public std::runtime_error {
public:
  explicit InvalidMessagePairCastException(const std::string& msg) : std::runtime_error(msg) {};
};
}