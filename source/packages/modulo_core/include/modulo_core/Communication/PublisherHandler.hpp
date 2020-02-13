/**
 * @author Baptiste Busch
 * @date 2019/06/14
 */

#ifndef MODULO_COMMUNICATION_PUBLISHER_HANDLER_H_
#define MODULO_COMMUNICATION_PUBLISHER_HANDLER_H_

#include <tuple>
#include <rclcpp/publisher.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include "modulo_core/Communication/MessagePassingCommunication.hpp"

namespace Modulo
{
	namespace Core
	{
		namespace Communication
		{
			using PublisherParameters = std::tuple<StateRepresentation::StateType, std::string, std::chrono::milliseconds>;

			/**
 		 	 * @class PublisherHandler
 		     * @brief Class to define a publisher
 		     * @tparam RecT the type of recipient (of StateRepresentation::State base class)
 		     * @tparam MsgT the type of associated ROS2 message
 		     *
 		     */
			template <class RecT, typename MsgT>
			class PublisherHandler : public MessagePassingCommunication
			{
			private:
				bool activated_; ///< indicate if the publisher is activated or not
				std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MsgT> > publisher_; ///< reference to the ROS2 publisher
				std::shared_ptr<rclcpp::TimerBase> timer_; ///< reference to the associated timer

			protected:
				/**
				 * @brief Function to publish the values of the recipient over the network
				 * @param recipient [description]
				 */
				void publish(const RecT& recipient);

			public:
				/**
				 * @brief Constructor of a PublisherHandler
				 * @param  channel   the channel associated to the publisher
				 * @param  recipient the recipent associated to the publisher
				 * @param  timeout   period before timeout
				 * @param  clock     reference to the Cell clock
				 * @param  mutex     reference to the Cell mutex
				 */
				explicit PublisherHandler(const std::string& channel, const std::shared_ptr<StateRepresentation::State>& recipient, const std::chrono::milliseconds& timeout, const std::shared_ptr<rclcpp::Clock>& clock, const std::shared_ptr<std::mutex>& mutex);
				
				/**
				 * @brief Function to publish periodically 
				 */
				void publish_callback();

				/**
				 * @brief Setter of the publisher reference
				 * @param publisher the reference to the publisher
				 */
				void set_publisher(const std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MsgT> > & publisher);

				/**
				 * @brief Setter of the timer reference
				 * @param timer the reference to the timer
				 */
				void set_timer(const std::shared_ptr<rclcpp::TimerBase> & timer);

				/**
				 * @brief Function to activate the publisher
				 */
				void activate();

				/**
				 * @brief Function to deactivate the publisher
				 */
				void deactivate();

				/**
				 * @brief Function to check if the publisher is timed out
				 */
				virtual void check_timeout();
			};

			template <class RecT, typename MsgT>
			PublisherHandler<RecT, MsgT>::PublisherHandler(const std::string& channel, const std::shared_ptr<StateRepresentation::State>& recipient, const std::chrono::milliseconds& timeout, const std::shared_ptr<rclcpp::Clock>& clock, const std::shared_ptr<std::mutex>& mutex):
			MessagePassingCommunication(CommunicationType::PUBLISHER, channel, recipient, timeout, clock, mutex),
			activated_(false)
			{}

			template <class RecT, typename MsgT>
			void PublisherHandler<RecT, MsgT>::publish(const RecT& recipient)
			{
				auto out_msg = std::make_unique<MsgT>();
				StateConversion::write_msg(*out_msg, recipient, this->get_clock().now());
				this->publisher_->publish(std::move(out_msg));
			}
			
			template <class RecT, typename MsgT>
			void PublisherHandler<RecT, MsgT>::publish_callback()
			{
				std::lock_guard<std::mutex> guard(this->get_mutex());
				if(!this->get_recipient().is_empty() && this->activated_)
				{	
					this->publish(static_cast<RecT&>(this->get_recipient()));
				}
			}

			template <class RecT, typename MsgT>
			inline void PublisherHandler<RecT, MsgT>::set_publisher(const std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MsgT> > & publisher)
			{
				this->publisher_ = std::move(publisher);
			}

			template <class RecT, typename MsgT>
			inline void PublisherHandler<RecT, MsgT>::set_timer(const std::shared_ptr<rclcpp::TimerBase> & timer)
			{
				this->timer_ = std::move(timer);
			}

			template <class RecT, typename MsgT>
			inline void PublisherHandler<RecT, MsgT>::activate()
			{
				this->activated_ = true;
				this->publisher_->on_activate();
			}

			template <class RecT, typename MsgT>
			inline void PublisherHandler<RecT, MsgT>::deactivate()
			{
				this->activated_ = false;
				this->publisher_->on_deactivate();
			}

			template <class RecT, typename MsgT>
			inline void PublisherHandler<RecT, MsgT>::check_timeout()
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