#pragma once

#include "modulo_core/exceptions/CoreException.hpp"

namespace modulo_core::exceptions {

/**
 * @class InvalidPointerCastException
 * @brief An exception class to notify if the result of getting an instance of a derived class through dynamic
 * down-casting of an object of the base class is not a correctly typed instance of the derived class.
 */
class InvalidPointerCastException : public CoreException {
public:
  explicit InvalidPointerCastException(const std::string& msg) : CoreException("InvalidPointerCastException", msg) {}
};
}// namespace modulo_core::exceptions


