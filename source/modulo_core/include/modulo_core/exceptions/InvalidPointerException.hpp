#pragma once

#include "modulo_core/exceptions/CoreException.hpp"

namespace modulo_core::exceptions {

/**
 * @class InvalidPointerException
 * @brief An exception class to notify if an object has no reference count (if the object is not owned by any pointer)
 * when attempting to get a derived instance through dynamic down-casting.
 */
class InvalidPointerException : public CoreException {
public:
  explicit InvalidPointerException(const std::string& msg) : CoreException(msg) {};
};
}// namespace modulo_core::exceptions

