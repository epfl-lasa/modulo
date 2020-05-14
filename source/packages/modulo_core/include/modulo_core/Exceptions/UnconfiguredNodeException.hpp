#pragma once

#include <iostream>
#include <exception>

namespace Modulo
{
	namespace Exceptions
	{
		class UnconfiguredNodeException: public std::runtime_error
		{
		public:
			explicit UnconfiguredNodeException(const std::string& msg) : runtime_error(msg)
			{};
		};
	}
}
