#pragma once

#include <stdexcept>
#include <string>

namespace modulo_new_core::exceptions {
class NullPointerException : public std::runtime_error {
public:
  explicit NullPointerException(const std::string& msg) : std::runtime_error(msg) {};
};
}