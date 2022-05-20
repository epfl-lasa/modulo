#pragma once

#include "modulo_components/exceptions/ComponentException.hpp"

namespace modulo_components::exceptions {
class ComponentParameterException : public ComponentException {
public:
  explicit ComponentParameterException(const std::string& msg) : ComponentException(msg) {};
};
}