/**
 * @author Baptiste Busch
 */
#pragma once

#include <iostream>
#include <exception>

namespace Modulo
{
	namespace Exceptions
	{
		class ParameterNotFoundException: public std::runtime_error
		{
		public:
			explicit ParameterNotFoundException(const std::string& parameter_name):
			runtime_error("Parameter " + parameter_name + " not found in the list of parameters")
			{};
		};
	}
}
