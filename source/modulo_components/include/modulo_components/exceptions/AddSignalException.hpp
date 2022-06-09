#pragma once

#include "modulo_components/exceptions/ComponentException.hpp"

namespace modulo_components::exceptions {

/**
 * @class AddSignalException
 * @brief An exception class to notify errors when adding a signal.
 * @details This is an exception class to be thrown if there is a
 * problem while adding a signal to the component.
 */
class AddSignalException : public ComponentException {
public:
  explicit AddSignalException(const std::string& msg) : ComponentException(msg) {};
};

}// namespace modulo_components::exceptions