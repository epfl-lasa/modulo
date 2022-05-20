#pragma once

#include "modulo_components/exceptions/ComponentException.hpp"

namespace modulo_components::exceptions {
class LookupTransformException : public ComponentException {
public:
  explicit LookupTransformException(const std::string& msg) : ComponentException(msg) {};
};
}