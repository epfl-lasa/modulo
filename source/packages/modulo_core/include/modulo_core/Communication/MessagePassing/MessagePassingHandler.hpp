/**
 * @author Baptiste Busch
 * @date 2020/02/13
 */

#ifndef MODULO_COMMUNICATION_MESSAGEPASSINGCOMMUNICATION_H_
#define MODULO_COMMUNICATION_MESSAGEPASSINGCOMMUNICATION_H_

#include <iostream>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "modulo_core/Communication/CommunicationHandler.hpp"
#include "modulo_core/Communication/MessagePassing/StateConversion.hpp"

namespace Modulo
{
	namespace Core
	{
		namespace Communication
		{
			namespace MessagePassing
			{
				/**
				 * @class CommunicationHandler
				 * @brief Abstract class to define a MessagePassingHandler type communication interface
				 */
				class MessagePassingHandler : public CommunicationHandler
				{
				private:
					std::shared_ptr<StateRepresentation::State> recipient_; ///< recipient associated to the handler

				public:
					/**
					 * @brief Constructor for a CommunicationHandler
					 * @param  type      the type of CommunicationHandler from the CommunicationType enumeration
					 * @param  recipient recipient associated to the handler
					 * @param  timeout   period before considered time out
					 * @param  mutex     reference to the Cell mutex
					 */
					explicit MessagePassingHandler(const CommunicationType& type,  const std::shared_ptr<StateRepresentation::State>& recipient, const std::chrono::milliseconds& timeout, const std::shared_ptr<std::mutex>& mutex);

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
				};

				inline const StateRepresentation::State& MessagePassingHandler::get_recipient() const
				{
					return *this->recipient_;
				}

				inline StateRepresentation::State& MessagePassingHandler::get_recipient()
				{
					return *this->recipient_;
				}
			}
		}
	}
}
#endif