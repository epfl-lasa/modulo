/**
 * @author Baptiste Busch
 * @date 2019/06/14
 *
 */

#ifndef MODULO_COMMUNICATION_SUBSCRIPTION_HANDLER_H_
#define MODULO_COMMUNICATION_SUBSCRIPTION_HANDLER_H_

#include "modulo_core/Communication/MessagePassingCommunication.hpp"

namespace Modulo
{
	namespace Core
	{
		namespace Communication
		{
			using SubscriptionParameters = std::tuple<StateRepresentation::StateType, std::string>;

			/**
 		 	 * @class SubscriptionHandler
 		     * @brief Class to define a subscription
 		     * @tparam RecT the type of recipient (of StateRepresentation::State base class)
 		     * @tparam MsgT the type of associated ROS2 message
 		     *
 		     */
			template <class RecT, typename MsgT>
			class SubscriptionHandler : public MessagePassingCommunication
			{
			private:
				std::shared_ptr<rclcpp::Subscription<MsgT> > subscription_; ///< reference to the ROS2 subscription

			public:
				/**
				 * @brief Constructor of a SubscriptionHandler
				 * @param  channel   the channel associated to the subscription
				 * @param  recipient the recipient associated to the subscription
				 * @param  timeout   the period before timeout
				 * @param  clock     reference to the Cell clock
				 * @param  mutex     reference to the Cell mutex
				 */
				explicit SubscriptionHandler(const std::string& channel, const std::shared_ptr<StateRepresentation::State>& recipient, const std::chrono::milliseconds& timeout, const std::shared_ptr<rclcpp::Clock>& clock, const std::shared_ptr<std::mutex>& mutex);
				
				/**
				 * @brief Callback function to receive the message from the network
				 * @param msg reference to the ROS message
				 */
				void subscription_callback(const std::shared_ptr<MsgT> msg);

				/**
				 * @brief Setter of the subscription reference
				 * @param subscription the reference to the subscription
				 */
				void set_subscription(const std::shared_ptr<rclcpp::Subscription<MsgT> > & subscription);
				
				/**
				 * @brief Function to activate the subscription
				 */
				void activate();

				/**
				 * @brief Function to deactivate the subscription
				 */
				void deactivate();

				/**
				 * @brief Function to check if the subscription is timed out
				 */
				void check_timeout();
			};

			template <class RecT, typename MsgT>
			SubscriptionHandler<RecT, MsgT>::SubscriptionHandler(const std::string& channel, const std::shared_ptr<StateRepresentation::State>& recipient, const std::chrono::milliseconds& timeout, const std::shared_ptr<rclcpp::Clock>& clock, const std::shared_ptr<std::mutex>& mutex):
			MessagePassingCommunication(CommunicationType::SUBSCRIPTION, channel, recipient, timeout, clock, mutex)
			{}
			
			template <class RecT, typename MsgT>
			void SubscriptionHandler<RecT, MsgT>::subscription_callback(const std::shared_ptr<MsgT> msg)
			{
				std::lock_guard<std::mutex> guard(this->get_mutex());
				StateConversion::read_msg(static_cast<RecT&>(this->get_recipient()), *msg);
			}

			template <class RecT, typename MsgT>
			inline void SubscriptionHandler<RecT, MsgT>::set_subscription(const std::shared_ptr<rclcpp::Subscription<MsgT> > & subscription)
			{
				this->subscription_ = std::move(subscription);
			}
			
			template <class RecT, typename MsgT>
			inline void SubscriptionHandler<RecT, MsgT>::activate()
			{}

			template <class RecT, typename MsgT>
			inline void SubscriptionHandler<RecT, MsgT>::deactivate()
			{}

			template <class RecT, typename MsgT>
			inline void SubscriptionHandler<RecT, MsgT>::check_timeout()
			{
				if(this->get_timeout() != std::chrono::milliseconds(0))
				{
					if(this->get_recipient().is_deprecated(this->get_timeout()))
					{
						this->get_recipient().initialize();
					}
				}
			}
		}
	}
}
#endif