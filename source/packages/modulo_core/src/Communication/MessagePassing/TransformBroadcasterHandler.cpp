#include "modulo_core/Communication/MessagePassing/TransformBroadcasterHandler.hpp"

namespace Modulo
{
	namespace Core
	{
		namespace Communication
		{
			namespace MessagePassing
			{
				TransformBroadcasterHandler::TransformBroadcasterHandler(const std::shared_ptr<StateRepresentation::CartesianPose>& recipient, const std::chrono::milliseconds& timeout, const std::shared_ptr<rclcpp::Clock>& clock, const std::shared_ptr<std::mutex>& mutex):
				PublisherHandler<StateRepresentation::CartesianPose, tf2_msgs::msg::TFMessage>(recipient, timeout, clock, mutex)
				{}

				TransformBroadcasterHandler::TransformBroadcasterHandler(const std::chrono::milliseconds& timeout, const std::shared_ptr<rclcpp::Clock>& clock, std::shared_ptr<std::mutex>& mutex):
				PublisherHandler<StateRepresentation::CartesianPose, tf2_msgs::msg::TFMessage>(std::make_shared<StateRepresentation::CartesianPose>(), timeout, clock, mutex)
				{}
			}
		}
	}
}