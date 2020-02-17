/**
 * @author Baptiste Busch
 * @date 2020/02/14
 *
 */
#ifndef MODULOCORE_EXCEPTIONS_SERVICENOTAVAILABLEEXCEPTION_H_
#define MODULOCORE_EXCEPTIONS_SERVICENOTAVAILABLEEXCEPTION_H_

#include <iostream>
#include <exception>

namespace Modulo
{
	namespace Exceptions
	{
		class ServiceNotAvailableException: public std::runtime_error
		{
		public:
			explicit ServiceNotAvailableException(const std::string& service_name):
			runtime_error("Service " + service_name + " is not available")
			{};
		};
	}
}
#endif