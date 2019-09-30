

#ifndef DYNAMICALSYSTEMS_EXCEPTIONS_NOTIMPLEMENTEDEXCEPTION_H_
#define DYNAMICALSYSTEMS_EXCEPTIONS_NOTIMPLEMENTEDEXCEPTION_H_

#include <iostream>
#include <exception>

namespace DynamicalSystems
{
	namespace Exceptions
	{
		class NotImplementedException : public std::logic_error
		{
		public:
			explicit NotImplementedException(const std::string& msg) : logic_error(msg)
			{};
		};
	}
}
#endif