#include "protocol_buffers/Communication.hpp"

namespace ProtocolBuffers
{
	Communication::Communication(const std::string& channel, const zmq::socket_type& socket_type, const std::string& ip, const std::string& port):
	channel_(channel), ip_(ip), port_(port), context_(1), socket_(context_, socket_type)
	{}

	void Communication::send(const std::string& message)
	{
		std::string text = this->channel_ + message;
		zmq::message_t msg(text.size());
		memcpy(msg.data(), text.c_str(), text.size());
		this->socket_.send(msg);
	}

	std::string Communication::receive()
	{
		
	}
}