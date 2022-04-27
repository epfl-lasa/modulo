#pragma once

#include <stdexcept>
#include <string>

namespace modulo_new_core::exceptions {
class NotLifecyclePublisherException : public std::runtime_error {
public:
  explicit NotLifecyclePublisherException(const std::string& msg) : std::runtime_error(msg) {};
};
}