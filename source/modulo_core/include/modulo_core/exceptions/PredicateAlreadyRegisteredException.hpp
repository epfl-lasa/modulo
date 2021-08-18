#pragma once

#include <exception>
#include <iostream>

namespace modulo::core::exceptions {
class PredicateAlreadyRegisteredException : public std::runtime_error {
public:
  explicit PredicateAlreadyRegisteredException(const std::string& predicate_name) : runtime_error("Predicate " + predicate_name + " is already registered in the list of predicates"){};
};
}// namespace modulo::core::exceptions
