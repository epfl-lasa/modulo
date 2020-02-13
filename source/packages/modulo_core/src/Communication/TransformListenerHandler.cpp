#include "modulo_core/Communication/TransformListenerHandler.hpp"

namespace Modulo
{
	namespace Core
	{
		namespace Communication
		{
			TransformListenerHandler::TransformListenerHandler(const std::shared_ptr<StateRepresentation::CartesianPose>& recipient, const std::chrono::milliseconds& timeout, const std::shared_ptr<rclcpp::Clock>& clock, const std::shared_ptr<std::mutex>& mutex):
			MessagePassingCommunication(CommunicationType::TRANSFORMLISTENER, "tf_listener", recipient, timeout, clock, mutex),
			buffer_(clock)
			{
				this->tf_listener_ = std::make_unique<tf2_ros::TransformListener>(buffer_);
			}

			TransformListenerHandler::TransformListenerHandler(const std::chrono::milliseconds& timeout, const std::shared_ptr<rclcpp::Clock>& clock, std::shared_ptr<std::mutex>& mutex):
			MessagePassingCommunication(CommunicationType::TRANSFORMLISTENER, "tf_listener", std::make_shared<StateRepresentation::CartesianPose>(), timeout, clock, mutex),
			buffer_(clock) 
			{
				this->tf_listener_ = std::make_unique<tf2_ros::TransformListener>(buffer_);
			}

			const StateRepresentation::CartesianPose TransformListenerHandler::lookup_transform(const std::string& frame_name, const std::string& reference_frame) const
			{
				geometry_msgs::msg::TransformStamped transformStamped;
				StateRepresentation::CartesianPose result(frame_name, reference_frame);
				transformStamped = this->buffer_.lookupTransform(reference_frame, frame_name, tf2::TimePoint(std::chrono::milliseconds(0)), tf2::Duration(this->get_timeout()));
				StateConversion::read_msg(result, transformStamped);
					return result;
			}
		}
	}
}