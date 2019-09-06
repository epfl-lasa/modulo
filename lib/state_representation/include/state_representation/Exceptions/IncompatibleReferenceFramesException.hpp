

#ifndef STATEREPRESENTATION_EXCEPTIONS_INCOMPATIBLEREFERENCEFRAMESEXCEPTION_H_
#define STATEREPRESENTATION_EXCEPTIONS_INCOMPATIBLEREFERENCEFRAMESEXCEPTION_H_

#include <iostream>
#include <exception>

namespace StateRepresentation
{
	namespace Exceptions
	{
		class IncompatibleReferenceFramesException: public std::logic_error
		{
		public:
			IncompatibleReferenceFramesException(const std::string& msg) : logic_error(msg)
			{};
		};
	}
}
#endif