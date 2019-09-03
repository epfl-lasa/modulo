/**
 * @class SubscriptionHandler
 * @brief Class to define a subscription
 * @author Baptiste Busch
 * @date 2019/06/14
 *
 */

#ifndef MODULO_COMMUNICATION_SUBSCRIPTION_HANDLER_H_
#define MODULO_COMMUNICATION_SUBSCRIPTION_HANDLER_H_

#include "modulo_core/Communication/CommunicationHandler.hpp"

namespace ModuloCore
{
	namespace Communication
	{
		template <class RecT, typename MsgT>
		class SubscriptionHandler: public CommunicationHandler
		{
		private:
			std::shared_ptr<rclcpp::Subscription<MsgT> > subscription_;

		public:
			explicit SubscriptionHandler(const std::string& channel, const std::shared_ptr<StateRepresentation::State>& recipient, const std::chrono::milliseconds& timeout, const std::shared_ptr<rclcpp::Clock>& clock, std::shared_ptr<std::mutex>& mutex):
			CommunicationHandler(channel, recipient, timeout, clock, mutex)
			{}
			
			void subscription_callback(const std::shared_ptr<MsgT> msg)
			{
				std::lock_guard<std::mutex> guard(this->get_mutex());
				StateConversion::update(static_cast<RecT&>(this->get_recipient()), *msg);
			}

			inline void set_subscription(const std::shared_ptr<rclcpp::Subscription<MsgT> > & subscription)
			{
				this->subscription_ = std::move(subscription);
			}
			
			inline void activate()
			{}

			inline void deactivate()
			{}

			inline void check_timeout()
			{
				if(this->get_recipient().is_deprecated(this->get_timeout()))
				{
					this->get_recipient().initialize();
				}
			}
		};
	}
}
#endif