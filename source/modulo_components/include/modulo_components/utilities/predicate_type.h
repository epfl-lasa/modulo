#pragma once

#include <variant>

namespace modulo_components::utilities {

typedef std::variant<bool, std::function<bool(void)>> PredicateType;

}// namespace modulo_components::utilities