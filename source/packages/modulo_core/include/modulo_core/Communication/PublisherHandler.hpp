/**
 * @class PublisherHandler
 * @brief Class to define a publisher
 * @author Baptiste Busch
 * @date 2019/06/14
 *
 */

#ifndef MODULO_COMMUNICATION_PUBLISHER_HANDLER_H_
#define MODULO_COMMUNICATION_PUBLISHER_HANDLER_H_

#include "modulo_core/Communication/CommunicationHandler.hpp"

namespace Modulo
{
	namespace Core
	{
		namespace Communication
		{
			template <class RecT, typename MsgT>
			class PublisherHandler: public CommunicationHandler
			{
			private:
				bool activated_;
				std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MsgT> > publisher_;
				std::shared_ptr<rclcpp::TimerBase> timer_;

			public:
				explicit PublisherHandler(const std::string& channel, const std::shared_ptr<StateRepresentation::State>& recipient, const std::chrono::milliseconds& timeout, const std::shared_ptr<rclcpp::Clock>& clock, const std::shared_ptr<std::mutex>& mutex):
				CommunicationHandler("publisher", channel, recipient, timeout, clock, mutex), activated_(false)
				{}

				void publish(const RecT& recipient)
				{
					auto out_msg = std::make_unique<MsgT>();
					StateConversion::write_msg(*out_msg, recipient, this->get_clock().now());
					this->publisher_->publish(std::move(out_msg));
				}
				
				void publish_callback()
				{
					std::lock_guard<std::mutex> guard(this->get_mutex());
					if(!this->get_recipient().is_empty() && this->activated_)
					{	
						this->publish(static_cast<RecT&>(this->get_recipient()));
					}
				}

				inline void set_publisher(const std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MsgT> > & publisher)
				{
					this->publisher_ = std::move(publisher);
				}

				inline void set_timer(const std::shared_ptr<rclcpp::TimerBase> & timer)
				{
					this->timer_ = std::move(timer);
				}

				inline void activate()
				{
					this->activated_ = true;
					this->publisher_->on_activate();
				}

				inline void deactivate()
				{
					this->activated_ = false;
					this->publisher_->on_deactivate();
				}

				virtual inline void check_timeout()
				{
					if(this->get_timeout() != std::chrono::milliseconds(0))
					{
						if(this->get_recipient().is_deprecated(this->get_timeout()))
						{
							this->get_recipient().initialize();
						}
					}
				}
			};
		}
	}
}
#endif