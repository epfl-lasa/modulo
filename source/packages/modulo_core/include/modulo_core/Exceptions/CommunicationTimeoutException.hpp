/**
 * @author Baptiste Busch
 * @date 2020/02/14
 *
 */
#pragma once

#include <iostream>
#include <exception>

namespace Modulo
{
	namespace Exceptions
	{
		class CommunicationTimeoutException: public std::runtime_error
		{
		public:
			explicit CommunicationTimeoutException(const std::string& service_name):
			runtime_error("Communication with service " + service_name + " timed out")
			{};
		};
	}
}
