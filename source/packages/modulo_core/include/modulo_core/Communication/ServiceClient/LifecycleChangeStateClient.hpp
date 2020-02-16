/**
 * @author Baptiste Busch
 * @date 2019/06/14
 *
 */
#ifndef MODULO_COMMUNICATION_SERVICECLIENT_LIFECYCLECHANGESTATECLIENT_H_
#define MODULO_COMMUNICATION_SERVICECLIENT_LIFECYCLECHANGESTATECLIENT_H_

#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include "modulo_core/Communication/ServiceClient/ClientHandler.hpp"

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
				 */
				class LifecycleChangeStateClient : public ClientHandler<lifecycle_msgs::srv::ChangeState>
				{
				private:
					std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> request_;

				public:
					/**
					 * @brief Constructor of a LifecycleChangeStateClient
					 * @param timeout period before timeout
					 * @param mutex reference to the Cell mutex
					 */
					explicit LifecycleChangeStateClient(const std::chrono::milliseconds& timeout, const std::shared_ptr<std::mutex>& mutex);

					void configure();

					void activate();

					void deactivate();

					void cleanup();
				};
			}
		}
	}
}
#endif