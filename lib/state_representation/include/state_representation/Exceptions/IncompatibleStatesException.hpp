

#ifndef MODULOCORE_EXCEPTIONS_INCOMPATIBLESTATESEXCEPTION_H_
#define MODULOCORE_EXCEPTIONS_INCOMPATIBLESTATESEXCEPTION_H_

#include <iostream>
#include <exception>

namespace StateRepresentation
{
	namespace Exceptions
	{
		class IncompatibleStatesException: public std::logic_error
		{
		public:
			IncompatibleStatesException(const std::string& msg) : logic_error(msg)
			{};
		};
	}
}
#endif