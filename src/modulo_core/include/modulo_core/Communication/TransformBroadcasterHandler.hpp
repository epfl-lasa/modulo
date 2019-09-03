/**
 * @class TransformBroadcasterHandler
 * @brief Class to define a transform broadcaster
 * @author Baptiste Busch
 * @date 2019/06/14
 *
 */

#ifndef MODULO_COMMUNICATION_TRANSFORMBROADCASTERHANDLER_H_
#define MODULO_COMMUNICATION_TRANSFORMBROADCASTERHANDLER_H_

#include "modulo_core/Communication/PublisherHandler.hpp"
#include "state_representation/Cartesian/CartesianPose.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace ModuloCore
{
	namespace Communication
	{
		class TransformBroadcasterHandler: public PublisherHandler<StateRepresentation::CartesianPose, tf2_msgs::msg::TFMessage>
		{
		public:
			explicit TransformBroadcasterHandler(const std::shared_ptr<StateRepresentation::CartesianPose>& recipient, const std::chrono::milliseconds& timeout, const std::shared_ptr<rclcpp::Clock>& clock, std::shared_ptr<std::mutex>& mutex):
			PublisherHandler<StateRepresentation::CartesianPose, tf2_msgs::msg::TFMessage>("tf", recipient, timeout, clock, mutex)
			{}

			explicit TransformBroadcasterHandler(const std::chrono::milliseconds& timeout, const std::shared_ptr<rclcpp::Clock>& clock, std::shared_ptr<std::mutex>& mutex):
			PublisherHandler<StateRepresentation::CartesianPose, tf2_msgs::msg::TFMessage>("tf", std::make_shared<StateRepresentation::CartesianPose>(), timeout, clock, mutex)
			{}

			inline void send_transform(const StateRepresentation::CartesianPose& transform)
			{
				static_cast<StateRepresentation::CartesianPose&>(this->get_recipient()) = transform;
			}
	    };
	}
}
#endif