#pragma once

#include <exception>
#include <iostream>

namespace modulo_components::exceptions {
class ImmutablePredicateException : public std::logic_error {
public:
  explicit ImmutablePredicateException(const std::string& predicate_name) :
      logic_error("Predicate " + predicate_name + " is a callback function whose value can't be set directly") {};
};
}// namespace modulo_components::exceptions
