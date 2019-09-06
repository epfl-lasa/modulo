

#ifndef STATEREPRESENTATION_EXCEPTIONS_INCOMPATIBLESIZEEXCEPTION_H_
#define STATEREPRESENTATION_EXCEPTIONS_INCOMPATIBLESIZEEXCEPTION_H_

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