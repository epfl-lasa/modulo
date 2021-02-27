#pragma once

#include <exception>
#include <iostream>

namespace modulo::core::exceptions {
class ParameterNotFoundException : public std::runtime_error {
public:
  explicit ParameterNotFoundException(const std::string& parameter_name) : runtime_error("Parameter " + parameter_name + " not found in the list of parameters"){};
};
}// namespace modulo::core::exceptions
