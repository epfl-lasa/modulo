#pragma once

#include <state_representation/exceptions/InvalidParameterException.hpp>

namespace modulo_new_core::exceptions {
class IncompatibleParameterException : public state_representation::exceptions::InvalidParameterException {
public:
  explicit IncompatibleParameterException(const std::string& msg) :
      state_representation::exceptions::InvalidParameterException(msg) {};
};
}