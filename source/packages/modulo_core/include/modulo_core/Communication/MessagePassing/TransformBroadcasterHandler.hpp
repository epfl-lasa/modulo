/**
 * @author Baptiste Busch
 * @date 2019/06/14
 */

#ifndef MODULO_COMMUNICATION_TRANSFORMBROADCASTERHANDLER_H_
#define MODULO_COMMUNICATION_TRANSFORMBROADCASTERHANDLER_H_

#include <tf2_msgs/msg/tf_message.hpp>
#include "modulo_core/Communication/MessagePassing/PublisherHandler.hpp"
#include "state_representation/Space/Cartesian/CartesianState.hpp"

namespace Modulo
{
	namespace Core
	{
		namespace Communication
		{
			namespace MessagePassing
			{
				/**
				 * @class TransformBroadcasterHandler
				 * @brief Class to define a transform broadcaster
				 */
				class TransformBroadcasterHandler: public PublisherHandler<StateRepresentation::CartesianState, tf2_msgs::msg::TFMessage>
				{
				public:
					/**
					 * @brief Constructor for and asychronous TransformBroadcaster handler
					 * @param  recipient the associated recipient to publish
					 * @param  timeout   period before timeout
					 * @param  clock     reference to the Cell clock
					 * @param  mutex     reference to the Cell mutex
					 */
					template <typename DurationT>
					explicit TransformBroadcasterHandler(const std::shared_ptr<StateRepresentation::CartesianState>& recipient, const std::chrono::duration<int64_t, DurationT>& timeout, const std::shared_ptr<rclcpp::Clock>& clock, const std::shared_ptr<std::mutex>& mutex);

					/**
					 * @brief Constructor for TransformBroadcaster handler without an associated recipient
					 * @param  timeout   period before timeout
					 * @param  clock     reference to the Cell clock
					 * @param  mutex     reference to the Cell mutex
					 */
					template <typename DurationT>
					explicit TransformBroadcasterHandler(const std::chrono::duration<int64_t, DurationT>& timeout, const std::shared_ptr<rclcpp::Clock>& clock, const std::shared_ptr<std::mutex>& mutex);

					/**
					 * @brief Function to send a transform over the network
					 * @param transform the transformation to send
					 */
					void send_transform(const StateRepresentation::CartesianState& transform);
			    };

			    template <typename DurationT>
			    TransformBroadcasterHandler::TransformBroadcasterHandler(const std::shared_ptr<StateRepresentation::CartesianState>& recipient, const std::chrono::duration<int64_t, DurationT>& timeout, const std::shared_ptr<rclcpp::Clock>& clock, const std::shared_ptr<std::mutex>& mutex):
				PublisherHandler<StateRepresentation::CartesianState, tf2_msgs::msg::TFMessage>(recipient, timeout, clock, mutex)
				{}

				template <typename DurationT>
				TransformBroadcasterHandler::TransformBroadcasterHandler(const std::chrono::duration<int64_t, DurationT>& timeout, const std::shared_ptr<rclcpp::Clock>& clock, const std::shared_ptr<std::mutex>& mutex):
				PublisherHandler<StateRepresentation::CartesianState, tf2_msgs::msg::TFMessage>(std::make_shared<StateRepresentation::CartesianState>(), timeout, clock, mutex)
				{}

				inline void TransformBroadcasterHandler::send_transform(const StateRepresentation::CartesianState& transform)
				{
					this->publish(transform);
				}
			}
		}
	}
}
#endif