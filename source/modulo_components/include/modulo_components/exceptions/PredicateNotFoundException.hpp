#pragma once

#include <exception>
#include <iostream>

namespace modulo_components::exceptions {
class PredicateNotFoundException : public std::runtime_error {
public:
  explicit PredicateNotFoundException(const std::string& predicate_name) :
      runtime_error("Predicate " + predicate_name + " not found in the list of predicates") {};
};
}// namespace modulo::core::exceptions
