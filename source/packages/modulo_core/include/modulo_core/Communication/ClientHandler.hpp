/**
 * @author Baptiste Busch
 * @date 2019/06/14
 *
 */
#ifndef MODULO_COMMUNICATION_CLIENTHANDLER_H_
#define MODULO_COMMUNICATION_CLIENTHANDLER_H_

#include "modulo_core/Communication/CommunicationHandler.hpp"
#include "modulo_core/Exceptions/CommunicationTimeoutException.hpp"

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

				template<typename WaitTimeT>
				static std::future_status wait_for_result(const std::shared_future<std::shared_ptr<typename srvT::Response> >& future_result, const WaitTimeT& time_to_wait);

				/**
				 * @brief Send a request to the server asymchronously withtout waiting for the answer
				 * @param request the request
				 * @return the response as a future pointer
				 */
				std::shared_future<std::shared_ptr<typename srvT::Response> > send_request(const std::shared_ptr<typename srvT::Request>& request) const;

				/**
				 * @brief Send a request to the server asymchronously and wait for the answer
				 * @param request the request
				 * @return the response as a shared pointer
				 */
				std::shared_ptr<typename srvT::Response> send_blocking_request(const std::shared_ptr<typename srvT::Request>& request) const;
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

			template<typename srvT> template<typename WaitTimeT>
			std::future_status ClientHandler<srvT>::wait_for_result(const std::shared_future<std::shared_ptr<typename srvT::Response> >& future_result, const WaitTimeT& time_to_wait)
			{
				auto end = std::chrono::steady_clock::now() + time_to_wait;
				std::chrono::milliseconds wait_period(100);
				std::future_status status = std::future_status::timeout;
				do
				{
					auto now = std::chrono::steady_clock::now();
					auto time_left = end - now;
					if (time_left <= std::chrono::seconds(0)) break;
					status = future_result.wait_for((time_left < wait_period) ? time_left : wait_period);
				} while (rclcpp::ok() && status != std::future_status::ready);
				return status;
			}

			template <typename srvT>
			std::shared_future<std::shared_ptr<typename srvT::Response> > ClientHandler<srvT>::send_request(const std::shared_ptr<typename srvT::Request>& request) const
			{
				return this->client_->async_send_request(request);
			}

			template <typename srvT>
			std::shared_ptr<typename srvT::Response> ClientHandler<srvT>::send_blocking_request(const std::shared_ptr<typename srvT::Request>& request) const
			{
				auto response = this->client_->async_send_request(request);
				std::future_status status = ClientHandler::wait_for_result(response, this->get_timeout());
				if (status != std::future_status::ready) throw Exceptions::CommunicationTimeoutException("Communication with the server timed out");
				return response.get();
			}
		}
	}
}
#endif