#include "modulo_core/Communication/ServiceClient/LifecycleChangeStateClient.hpp"

namespace Modulo
{
	namespace Core
	{
		namespace Communication
		{
			namespace ServiceClient
			{
				LifecycleChangeStateClient::LifecycleChangeStateClient(const std::chrono::milliseconds& timeout, const std::shared_ptr<std::mutex>& mutex):
				ClientHandler<lifecycle_msgs::srv::ChangeState>(timeout, mutex),
				request_(std::make_shared<lifecycle_msgs::srv::ChangeState::Request>())
				{}

				void LifecycleChangeStateClient::configure()
				{
					this->request_->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
					auto response = this->send_request(this->request_);
				}

				void LifecycleChangeStateClient::activate()
				{
					this->request_->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
					auto response = this->send_request(this->request_);
				}

				void LifecycleChangeStateClient::deactivate()
				{
					this->request_->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
					auto response = this->send_request(this->request_);
				}

				void LifecycleChangeStateClient::cleanup()
				{
					this->request_->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;
					auto response = this->send_request(this->request_);
				}
			}
		}
	}
}