#pragma once

#include "modulo_core/exceptions/CoreException.hpp"

namespace modulo_core::exceptions {

/**
 * @class NullPointerException
 * @brief An exception class to notify that a certain pointer is null.
 * @details This is an exception class to be thrown if a pointer is null or is trying to be set to a null pointer.
 */
class NullPointerException : public CoreException {
public:
  explicit NullPointerException(const std::string& msg) : CoreException("NullPointerException", msg) {}
};
}// namespace modulo_core::exceptions

