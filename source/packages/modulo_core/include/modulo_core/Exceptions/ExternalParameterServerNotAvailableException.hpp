#pragma once

#include <exception>
#include <iostream>

namespace modulo::core::exceptions {
class ExternalParameterServerNotAvailableException : public std::runtime_error {
public:
  explicit ExternalParameterServerNotAvailableException(const std::string& node_name) :
  	runtime_error("Communication with parameter server on node " + node_name + " timed out"){};
};
}// namespace modulo::core::exceptions
