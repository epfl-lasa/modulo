#pragma once

#include <exception>
#include <iostream>

namespace modulo::core::exceptions {
class CommunicationTimeoutException : public std::runtime_error {
public:
  explicit CommunicationTimeoutException(const std::string& service_name) :
  	runtime_error("Communication with service " + service_name + " timed out"){};
};
}// namespace modulo::core::exceptions
