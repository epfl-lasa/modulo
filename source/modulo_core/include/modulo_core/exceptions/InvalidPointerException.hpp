#pragma once

#include <stdexcept>
#include <string>

namespace modulo_core::exceptions {

/**
 * @class InvalidPointerException
 * @brief An exception class to notify if an object has no reference count (if the object is not owned by any pointer)
 * when attempting to get a derived instance through dynamic down-casting.
 */
class InvalidPointerException : public std::runtime_error {
public:
  explicit InvalidPointerException(const std::string& msg) : std::runtime_error(msg) {};
};
}// namespace modulo_core::exceptions
