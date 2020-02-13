/**
 * @author Baptiste Busch
 * @date 2019/06/14
 *
 */
#ifndef MODULO_COMMUNICATION_CLIENTHANDLER_H_
#define MODULO_COMMUNICATION_CLIENTHANDLER_H_

#include "modulo_core/Communication/CommunicationHandler.hpp"

namespace Modulo
{
	namespace Core
	{
		namespace Communication
		{
			/**
			 * @class ClientHandler
			 * @brief Class to define a client
			 * @tparam srvT the type of service
			 */
			template <typename srvT>
			class ClientHandler : public CommunicationHandler
			{
			private:
				std::shared_ptr<rclcpp::Client<srvT> > client_; ///< reference to the ROS2 client

			public:
				/**
				 * @brief Constructor of a ClientHandler
				 * @param timeout period before timeout
				 * @param mutex reference to the Cell mutex
				 */
				explicit ClientHandler(const std::chrono::milliseconds& timeout, const std::shared_ptr<std::mutex>& mutex);

				/**
				 * @brief Setter of the publisher reference
				 * @param publisher the reference to the publisher
				 */
				void set_client(const std::shared_ptr<rclcpp::Client<srvT> > & client);

				/**
				 * @brief Send a requestion to the server
				 * @param request the request
				 * @return the response
				 */
				decltype(auto) send_request(const std::shared_ptr<typename srvT::Request>& request) const;
			};

			template <typename srvT>
			ClientHandler<srvT>::ClientHandler(const std::chrono::milliseconds& timeout, const std::shared_ptr<std::mutex>& mutex):
			CommunicationHandler(CommunicationType::CLIENT, timeout, mutex)
			{}

			template <typename srvT>
			inline void ClientHandler<srvT>::set_client(const std::shared_ptr<rclcpp::Client<srvT> > & client)
			{
				this->client_ = std::move(client);
			}

			template <typename srvT>
			inline decltype(auto) ClientHandler<srvT>::send_request(const std::shared_ptr<typename srvT::Request>& request) const
			{
				return this->client_->async_send_request(request);
			}
		}
	}
}
#endif