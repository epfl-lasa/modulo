

#ifndef STATEREPRESENTATION_EXCEPTIONS_INCOMPATIBLESTATESEXCEPTION_H_
#define STATEREPRESENTATION_EXCEPTIONS_INCOMPATIBLESTATESEXCEPTION_H_

#include <iostream>
#include <exception>

namespace StateRepresentation
{
	namespace Exceptions
	{
		class IncompatibleStatesException: public std::logic_error
		{
		public:
			explicit IncompatibleStatesException(const std::string& msg) : logic_error(msg)
			{};
		};
	}
}
#endif