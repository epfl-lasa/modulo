/**
 * @author Baptiste Busch
 * @date 2019/06/14
 *
 */
#pragma once

#include "modulo_core/Communication/CommunicationHandler.hpp"
#include "modulo_core/Exceptions/CommunicationTimeoutException.hpp"
#include "modulo_core/Exceptions/ServiceNotAvailableException.hpp"

namespace Modulo
{
	namespace Core
	{
		namespace Communication
		{
			namespace ServiceClient
			{
				/**
				 * @class ClientHandler
				 * @brief Class to define a client
				 * @tparam SrvT the type of service
				 */
				template <typename SrvT>
				class ClientHandler : public CommunicationHandler
				{
				private:
					std::shared_ptr<rclcpp::Client<SrvT> > client_; ///< reference to the ROS2 client

				public:
					/**
					 * @brief Constructor of a ClientHandler
					 * @param timeout period before timeout
					 * @param mutex reference to the Cell mutex
					 */
					template <typename DurationT>
					explicit ClientHandler(const std::chrono::duration<int64_t, DurationT>& timeout, const std::shared_ptr<std::mutex>& mutex);

					/**
					 * @brief Setter of the publisher reference
					 * @param publisher the reference to the publisher
					 */
					void set_client(const std::shared_ptr<rclcpp::Client<SrvT> > & client);

					/**
					 * @brief Static function to wait for the result of a request
					 * @tparam WaitTimeT a period to wait in std::chrono
					 * @param future_result the future result as a shared_future
					 * @param time_to_wait the maximum time to wait for the result
					 * @return the status of the waiting if the waited time is greater than the allowed time the status is timeout
					 */
					template <typename DurationT>
					static std::future_status wait_for_result(const std::shared_future<std::shared_ptr<typename SrvT::Response> >& future_result, const std::chrono::duration<int64_t, DurationT>& time_to_wait);

					/**
					 * @brief Send a request to the server asymchronously withtout waiting for the answer
					 * @param request the request
					 * @return the response as a future pointer
					 */
					std::shared_future<std::shared_ptr<typename SrvT::Response> > send_request(const std::shared_ptr<typename SrvT::Request>& request) const;

					/**
					 * @brief Send a request to the server asymchronously and wait for the answer
					 * @param request the request
					 * @return the response as a shared pointer
					 */
					std::shared_ptr<typename SrvT::Response> send_blocking_request(const std::shared_ptr<typename SrvT::Request>& request) const;
				};

				template <typename SrvT> template <typename DurationT>
				ClientHandler<SrvT>::ClientHandler(const std::chrono::duration<int64_t, DurationT>& timeout, const std::shared_ptr<std::mutex>& mutex):
				CommunicationHandler(CommunicationType::CLIENT, timeout, mutex)
				{}

				template <typename SrvT>
				inline void ClientHandler<SrvT>::set_client(const std::shared_ptr<rclcpp::Client<SrvT> > & client)
				{
					this->client_ = std::move(client);
					if (!this->client_->wait_for_service(this->get_timeout())) 
					{
						throw Exceptions::ServiceNotAvailableException(this->client_->get_service_name());
					}
				}

				template<typename SrvT> template <typename DurationT>
				std::future_status ClientHandler<SrvT>::wait_for_result(const std::shared_future<std::shared_ptr<typename SrvT::Response> >& future_result, const std::chrono::duration<int64_t, DurationT>& time_to_wait)
				{
					auto end = std::chrono::steady_clock::now() + time_to_wait;
					std::chrono::milliseconds wait_period(1);
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

				template <typename SrvT>
				std::shared_future<std::shared_ptr<typename SrvT::Response> > ClientHandler<SrvT>::send_request(const std::shared_ptr<typename SrvT::Request>& request) const
				{
					if (!this->client_->wait_for_service(this->get_timeout())) 
					{
						throw Exceptions::ServiceNotAvailableException(this->client_->get_service_name());
					}
					return this->client_->async_send_request(request);
				}

				template <typename SrvT>
				std::shared_ptr<typename SrvT::Response> ClientHandler<SrvT>::send_blocking_request(const std::shared_ptr<typename SrvT::Request>& request) const
				{
					if (!this->client_->wait_for_service(this->get_timeout())) 
					{
						throw Exceptions::ServiceNotAvailableException(this->client_->get_service_name());
					}
					auto response = this->client_->async_send_request(request);
					std::future_status status = ClientHandler::wait_for_result(response, this->get_timeout());
					if (status != std::future_status::ready)
					{
						throw Exceptions::CommunicationTimeoutException(this->client_->get_service_name());
					}
					return response.get();
				}
			}
		}
	}
}
