#pragma once

#include <exception>
#include <iostream>

namespace modulo::core::exceptions {
class UnconfiguredNodeException : public std::runtime_error {
public:
  explicit UnconfiguredNodeException(const std::string& msg) : runtime_error(msg){};
};
}// namespace modulo::core::exceptions
