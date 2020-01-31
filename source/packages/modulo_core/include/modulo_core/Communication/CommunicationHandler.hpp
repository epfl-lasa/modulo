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
			enum class CommunicationType
			{
				PUBLISHER,
				SUBSCRIPTION,
				TRANSFORMLISTENER
			};

			class CommunicationHandler
			{
			private:
				const CommunicationType type_; ///< type of the handler from the CommunicationType enumeration
				const std::string channel_; ///< channel associated to the handler
				std::shared_ptr<StateRepresentation::State> recipient_; ///< recipient associated to the handler
				std::chrono::milliseconds timeout_; ///< period before considered time out
				std::shared_ptr<rclcpp::Clock> clock_; ///< reference to the Cell clock
				std::shared_ptr<std::mutex> mutex_; ///< reference to the Cell mutex

			public:
				/**
				 * @brief Constructor for a CommunicationHandler
				 * @param  type      the type of CommunicationHandler from the CommunicationType enumeration
				 * @param  channel   channel associated to the handler
				 * @param  recipient recipient associated to the handler
				 * @param  timeout   period before considered time out
				 * @param  clock     reference to the Cell clock
				 * @param  mutex     reference to the Cell mutex
				 */
				explicit CommunicationHandler(const CommunicationType& type, const std::string& channel, const std::shared_ptr<StateRepresentation::State>& recipient, const std::chrono::milliseconds& timeout, const std::shared_ptr<rclcpp::Clock>& clock, const std::shared_ptr<std::mutex>& mutex);

				/**
				 * @brief Getter of the CommunicationType
				 * @return the type of the handler
				 */
				const CommunicationType& get_type();
			
				/**
				 * @brief Getter of the recipient
				 * @return the recipient
				 */
				const StateRepresentation::State& get_recipient() const;
			
				/**
				 * @brief Getter of the recipient as a non const value
				 * @return the recipient
				 */
				StateRepresentation::State& get_recipient();
				
				/**
				 * @brief Getter of the channel name
				 * @return the channel name
				 */
				const std::string& get_channel() const;
			
				/**
				 * @brief Getter of the timeout period
				 * @return the timeout period
				 */
				const std::chrono::milliseconds& get_timeout() const;
			
				/**
				 * @brief Getter of the clock reference
				 * @return the clock reference
				 */
				const rclcpp::Clock& get_clock() const;

				/**
				 * @brief Getter of the clock as a non const reference
				 * @return the clock reference
				 */
				rclcpp::Clock& get_clock();

				/**
				 * @brief Getter of the mutex reference
				 * @return the mutex reference
				 */
				std::mutex& get_mutex();
			
				/**
				 * @brief Virtual function to activate a handler
				 */
				virtual void activate();
			
				/**
				 * @brief Virtual function to deactivate a handler
				 */
				virtual void deactivate();
			
				/**
				 * @brief Virtual function to check if the handler is timed out
				 */
				virtual void check_timeout();
			};

			inline const CommunicationType& CommunicationHandler::get_type()
			{
				return this->type_;
			}

			inline const StateRepresentation::State& CommunicationHandler::get_recipient() const
			{
				return *this->recipient_;
			}

			inline StateRepresentation::State& CommunicationHandler::get_recipient()
			{
				return *this->recipient_;
			}

			inline const std::string& CommunicationHandler::get_channel() const
			{
				return this->channel_;
			}

			inline const std::chrono::milliseconds& CommunicationHandler::get_timeout() const
			{
				return this->timeout_;
			}

			inline const rclcpp::Clock& CommunicationHandler::get_clock() const
			{
				return *this->clock_;
			}

			inline rclcpp::Clock& CommunicationHandler::get_clock()
			{
				return *this->clock_;
			}

			inline std::mutex& CommunicationHandler::get_mutex()
			{
				return *this->mutex_;
			}

			inline void CommunicationHandler::activate()
			{}

			inline void CommunicationHandler::deactivate()
			{}

			inline void CommunicationHandler::check_timeout()
			{}
		}
	}
}
#endif