
#ifndef PROTOCOL_BUFFERS_COMMUNICATION_H_
#define PROTOCOL_BUFFERS_COMMUNICATION_H_

#include <zmq.hpp>
#include <string>


namespace ProtocolBuffers
{
	class Communication
	{
	protected:
		const std::string channel_;
		const std::string ip_;
		const std::string port_;
		zmq::context_t context_;
   		zmq::socket_t socket_;

   	public:
   		explicit Communication(const std::string& channel, const zmq::socket_type& socket_type, const std::string& ip="localhost", const std::string& port="5556");

   		virtual void send(const std::string& message);

   		virtual std::string receive();
	};
}

#endif