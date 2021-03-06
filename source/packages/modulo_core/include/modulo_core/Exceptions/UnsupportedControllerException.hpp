/**
 * @class UnsupportedControllerException
 * @brief Exception to raise when a controller is not supported on a robot
 * @author Baptiste Busch
 * @date 2019/09/20
 */

#pragma once

#include <iostream>
#include <exception>

namespace Modulo
{
	namespace Exceptions
	{
		class UnsupportedControllerException : public std::logic_error
		{
		public:
			explicit UnsupportedControllerException(const std::string& msg) : logic_error(msg)
			{};
		};
	}
}