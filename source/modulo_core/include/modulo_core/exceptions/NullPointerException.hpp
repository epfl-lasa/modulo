#pragma once

#include <stdexcept>
#include <string>

namespace modulo_core::exceptions {

/**
 * @class NullPointerException
 * @brief An exception class to notify that a certain pointer is null.
 * @details This is an exception class to be thrown if a pointer is null or is trying to be set to a null pointer.
 */
class NullPointerException : public std::runtime_error {
public:
  explicit NullPointerException(const std::string& msg) : std::runtime_error(msg) {};
};
}// namespace modulo_core::exceptions
