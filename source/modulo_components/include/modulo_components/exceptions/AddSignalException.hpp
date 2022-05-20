#pragma once

#include "modulo_components/exceptions/ComponentException.hpp"

namespace modulo_components::exceptions {
class AddSignalException : public ComponentException {
public:
  explicit AddSignalException(const std::string& msg) : ComponentException(msg) {};
};
}