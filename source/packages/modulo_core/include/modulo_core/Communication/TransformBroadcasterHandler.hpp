/**
 * @author Baptiste Busch
 * @date 2019/06/14
 */

#ifndef MODULO_COMMUNICATION_TRANSFORMBROADCASTERHANDLER_H_
#define MODULO_COMMUNICATION_TRANSFORMBROADCASTERHANDLER_H_

#include <tf2_msgs/msg/tf_message.hpp>
#include "modulo_core/Communication/PublisherHandler.hpp"
#include "state_representation/Space/Cartesian/CartesianPose.hpp"

namespace Modulo
{
	namespace Core
	{
		namespace Communication
		{
			/**
			 * @class TransformBroadcasterHandler
			 * @brief Class to define a transform broadcaster
			 */
			class TransformBroadcasterHandler: public PublisherHandler<StateRepresentation::CartesianPose, tf2_msgs::msg::TFMessage>
			{
			public:
				/**
				 * @brief Constructor for and asychronous TransformBroadcaster handler
				 * @param  recipient the associated recipient to publish
				 * @param  timeout   period before timeout
				 * @param  clock     reference to the Cell clock
				 * @param  mutex     reference to the Cell mutex
				 */
				explicit TransformBroadcasterHandler(const std::shared_ptr<StateRepresentation::CartesianPose>& recipient, const std::chrono::milliseconds& timeout, const std::shared_ptr<rclcpp::Clock>& clock, const std::shared_ptr<std::mutex>& mutex);

				/**
				 * @brief Constructor for TransformBroadcaster handler without an associated recipient
				 * @param  timeout   period before timeout
				 * @param  clock     reference to the Cell clock
				 * @param  mutex     reference to the Cell mutex
				 */
				explicit TransformBroadcasterHandler(const std::chrono::milliseconds& timeout, const std::shared_ptr<rclcpp::Clock>& clock, std::shared_ptr<std::mutex>& mutex);

				/**
				 * @brief Function to send a transform over the network
				 * @param transform the transformation to send
				 */
				void send_transform(const StateRepresentation::CartesianPose& transform);
		    };

			inline void TransformBroadcasterHandler::send_transform(const StateRepresentation::CartesianPose& transform)
			{
				this->publish(transform);
			}
		}
	}
}
#endif