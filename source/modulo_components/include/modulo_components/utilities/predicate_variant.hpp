#pragma once

#include <variant>

namespace modulo_components::utilities {

typedef std::variant<bool, std::function<bool(void)>> PredicateVariant;

}// namespace modulo_components::utilities
