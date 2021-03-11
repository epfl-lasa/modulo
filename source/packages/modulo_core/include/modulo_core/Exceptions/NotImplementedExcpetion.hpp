#pragma once

#include <exception>
#include <iostream>

namespace modulo::core::exceptions {
class NotImplementedException : public std::logic_error {
public:
  explicit NotImplementedException(const std::string& msg) : logic_error(msg){};
};
}// namespace modulo::core::exceptions
