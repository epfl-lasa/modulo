#pragma once

#include <stdexcept>
#include <string>

namespace modulo_new_core::exceptions {

/**
 * @class InvalidPointerCastException
 * @brief An exception class to notify if the result of getting an instance of a derived class through dynamic
 * down-casting of an object of the base class is not a correctly typed instance of the derived class.
 */
class InvalidPointerCastException : public std::runtime_error {
public:
  explicit InvalidPointerCastException(const std::string& msg) : std::runtime_error(msg) {};
};
}// namespace modulo_new_core::exceptions
