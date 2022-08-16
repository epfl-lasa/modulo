#pragma once

#include "modulo_components/exceptions/ComponentException.hpp"

namespace modulo_components::exceptions {

/**
 * @class LookupTransformException
 * @brief An exception class to notify an error while looking up TF transforms.
 * @details This is an exception class to be thrown if there is a problem with looking up a TF transform
 * (unconfigured buffer/listener, TF2 exception).
 */
class LookupTransformException : public ComponentException {
public:
  explicit LookupTransformException(const std::string& msg) : ComponentException("LookupTransformException", msg) {}
};
}// namespace modulo_components::exceptions
