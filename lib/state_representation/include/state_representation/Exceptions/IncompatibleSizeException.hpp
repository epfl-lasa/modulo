

#ifndef MODULOCORE_EXCEPTIONS_INCOMPATIBLESIZEEXCEPTION_H_
#define MODULOCORE_EXCEPTIONS_INCOMPATIBLESIZEEXCEPTION_H_

#include <iostream>
#include <exception>

namespace StateRepresentation
{
	namespace Exceptions
	{
		class IncompatibleSizeException: public std::logic_error
		{
		public:
			IncompatibleSizeException(const std::string& msg) : logic_error(msg)
			{};
		};
	}
}
#endif