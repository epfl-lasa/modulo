#pragma once

#include <exception>
#include <iostream>

namespace modulo::core::exceptions {
class ServiceNotAvailableException : public std::runtime_error {
public:
  explicit ServiceNotAvailableException(const std::string& service_name) : runtime_error("Service " + service_name + " is not available"){};
};
}// namespace modulo::core::exceptions
