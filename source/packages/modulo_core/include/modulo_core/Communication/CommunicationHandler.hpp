/**
 * @class CommunicationHandler
 * @brief Abstract class to define a Communication interface
 * @author Baptiste Busch
 * @date 2019/06/14
 *
 */

#ifndef MODULO_COMMUNICATION_COMMUNICATION_HANDLER_H_
#define MODULO_COMMUNICATION_COMMUNICATION_HANDLER_H_

#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "modulo_core/Communication/StateConversion.hpp"

namespace Modulo
{
	namespace Core
	{
		namespace Communication
		{
			class CommunicationHandler
			{
			private:
				const std::string type_;
				std::string channel_;
				std::shared_ptr<StateRepresentation::State> recipient_;
				std::chrono::milliseconds timeout_;
				std::shared_ptr<rclcpp::Clock> clock_;
				std::shared_ptr<std::mutex> mutex_;

			public:
				explicit CommunicationHandler(const std::string& type, const std::string& channel, const std::shared_ptr<StateRepresentation::State>& recipient, const std::chrono::milliseconds& timeout, const std::shared_ptr<rclcpp::Clock>& clock, const std::shared_ptr<std::mutex>& mutex):
				type_(type), channel_(channel), recipient_(recipient), timeout_(timeout), clock_(clock), mutex_(mutex)
				{}

				inline const std::string& get_type()
				{
					return this->type_;
				}

				inline const StateRepresentation::State& get_recipient() const
				{
					return *this->recipient_;
				}

				inline StateRepresentation::State& get_recipient()
				{
					return *this->recipient_;
				}

				inline const std::string& get_channel() const
				{
					return this->channel_;
				}

				inline const std::chrono::milliseconds& get_timeout() const
				{
					return this->timeout_;
				}

				inline const rclcpp::Clock& get_clock() const
				{
					return *this->clock_;
				}

				inline rclcpp::Clock& get_clock()
				{
					return *this->clock_;
				}

				inline std::mutex& get_mutex()
				{
					return *this->mutex_;
				}

				inline virtual void activate()
				{}

				inline virtual void deactivate()
				{}

				inline virtual void check_timeout()
				{}
			};
		}
	}
}
#endif