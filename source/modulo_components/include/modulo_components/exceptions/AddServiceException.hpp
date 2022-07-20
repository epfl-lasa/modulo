#pragma once

#include "modulo_components/exceptions/ComponentException.hpp"

namespace modulo_components::exceptions {

/**
 * @class AddServiceException
 * @brief An exception class to notify errors when adding a service.
 * @details This is an exception class to be thrown if there is a problem while adding a service to the component.
 */
class AddServiceException : public ComponentException {
public:
  explicit AddServiceException(const std::string& msg) : ComponentException(msg) {};
};
}// namespace modulo_components::exceptions
