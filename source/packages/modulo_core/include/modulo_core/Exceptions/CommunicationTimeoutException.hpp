/**
 * @author Baptiste Busch
 * @date 2020/02/14
 *
 */
#ifndef MODULOCORE_EXCEPTIONS_COMMUNICATIONTIMEOUTEXCEPTION_H_
#define MODULOCORE_EXCEPTIONS_COMMUNICATIONTIMEOUTEXCEPTION_H_

#include <iostream>
#include <exception>

namespace Modulo
{
	namespace Exceptions
	{
		class CommunicationTimeoutException: public std::runtime_error
		{
		public:
			explicit CommunicationTimeoutException(const std::string& msg) : runtime_error(msg)
			{};
		};
	}
}
#endif