#pragma once

#include "modulo_core/exceptions/CoreException.hpp"

namespace modulo_core::exceptions {

/**
 * @class MessageTranslationException
 * @brief An exception class to notify that the translation of a ROS message failed.
 */
class MessageTranslationException : public CoreException {
public:
  explicit MessageTranslationException(const std::string& msg) : CoreException(msg) {};
};
}// namespace modulo_core::exceptions


