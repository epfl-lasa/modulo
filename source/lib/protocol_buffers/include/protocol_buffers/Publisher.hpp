
#ifndef PROTOCOL_BUFFERS_PUBLISHER_H_
#define PROTOCOL_BUFFERS_PUBLISHER_H_

#include "protocol_buffers/Communication.hpp"

namespace ProtocolBuffers
{
	template<class O>
	class Publisher: Communication
	{
   	public:
   		explicit Publisher(const std::string& channel, const std::string& port="5556");

   		void send(const O& object);
	};

	template<class O>
	Publisher<O>::Publisher(const std::string& channel, const std::string& port):
	Communication(channel, zmq::socket_type::pub, "*", port)
	{
		this->socket_.bind("tcp://" + this->ip_ + ":" + this->port_);
	}

	template<class O>
	void Publisher<O>::send(const O& object)
	{
		this->Communication::send(object.serialize());
	}

	template<>
	void Publisher<std::string>::send(const std::string& message)
	{
		this->Communication::send(message);
	}
}

#endif