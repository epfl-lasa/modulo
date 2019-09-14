

#ifndef MODULOCORE_EXCEPTIONS_UNCONFIGUREDNODEEXCEPTION_H_
#define MODULOCORE_EXCEPTIONS_UNCONFIGUREDNODEEXCEPTION_H_

#include <iostream>
#include <exception>

namespace Modulo
{
	namespace Exceptions
	{
		class UnconfiguredNodeException: public std::runtime_error
		{
		public:
			UnconfiguredNodeException(const std::string& msg) : runtime_error(msg)
			{};
		};
	}
}
#endif