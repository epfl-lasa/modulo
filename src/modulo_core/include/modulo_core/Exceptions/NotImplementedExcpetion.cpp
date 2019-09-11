/**
 * @class NotImplementedException
 * @brief Exception to raise when a functionnality is not implemented
 * @author Baptiste Busch
 * @date 2019/09/11
 */

#ifndef MODULOCORE_EXCEPTIONS_NOTIMPLEMENTEDEXCEPTION_H_
#define MODULOCORE_EXCEPTIONS_NOTIMPLEMENTEDEXCEPTION_H_

#include <iostream>
#include <exception>

namespace Modulo
{
	namespace Exceptions
	{
		class NotImplementedException : public std::logic_error
		{
		public:
			NotImplementedException(const std::string& msg) : logic_error(msg)
			{};
		};
	}
}
#endif