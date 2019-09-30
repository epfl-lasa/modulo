
#ifndef PROTOCOL_BUFFERS_SUBSCRIBER_H_
#define PROTOCOL_BUFFERS_SUBSCRIBER_H_

#include "protocol_buffers/Communication.hpp"

namespace ProtocolBuffers
{
	template<class S>
	class Subscriber: Communication
	{
   	public:
   		explicit Subscriber(const std::string& channel, const std::string& port="5556");

   		void send(const S& message);
	};

	template<class S>
	Subscriber<S>::Subscriber(const std::string& channel, const std::string& port):
	Communication(channel, zmq::socket_type::pub, "*", port)
	{
		this->socket_.bind("tcp://" + this->ip_ + ":" + this->port_);
	}

	template<class S>
	S Subscriber<S>receive()
	{
		std::string msg_str;
		message.SerializeToString(&msg_str);
		this->Communication::send(msg_str);
	}

	template<>
	std::string Subscriber<std::string>::receive()
	{
		this->Communication::send(message);
	}
}

#endif