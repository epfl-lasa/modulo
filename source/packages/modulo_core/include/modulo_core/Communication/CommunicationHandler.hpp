/**
 * @author Baptiste Busch
 * @date 2019/06/14
 */

#ifndef MODULO_COMMUNICATION_COMMUNICATION_HANDLER_H_
#define MODULO_COMMUNICATION_COMMUNICATION_HANDLER_H_

#include <iostream>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace Modulo
{
	namespace Core
	{
		namespace Communication
		{
			/**
			 * @enum CommunicationType
			 * @brief enumeration of the possible type of communication interfaces
			 */
			enum class CommunicationType
			{
				PUBLISHER,
				SUBSCRIPTION,
				CLIENT,
				TRANSFORMLISTENER
			};

			/**
			 * @class CommunicationHandler
			 * @brief Abstract class to define a Communication interface
			 */
			class CommunicationHandler
			{
			private:
				const CommunicationType type_; ///< type of the handler from the CommunicationType enumeration
				std::chrono::milliseconds timeout_; ///< period before considered time out
				std::shared_ptr<std::mutex> mutex_; ///< reference to the Cell mutex

			public:
				/**
				 * @brief Constructor for a CommunicationHandler
				 * @param  type      the type of CommunicationHandler from the CommunicationType enumeration
				 * @param  timeout   period before considered time out
				 * @param  clock     reference to the Cell clock
				 * @param  mutex     reference to the Cell mutex
				 */
				explicit CommunicationHandler(const CommunicationType& type, const std::chrono::milliseconds& timeout, const std::shared_ptr<std::mutex>& mutex);

				/**
				 * @brief Getter of the CommunicationType
				 * @return the type of the handler
				 */
				const CommunicationType& get_type();
			
				/**
				 * @brief Getter of the timeout period
				 * @return the timeout period
				 */
				const std::chrono::milliseconds& get_timeout() const;

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

			inline const std::chrono::milliseconds& CommunicationHandler::get_timeout() const
			{
				return this->timeout_;
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