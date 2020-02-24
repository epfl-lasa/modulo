#include "modulo_core/Communication/MessagePassing/TransformListenerHandler.hpp"

namespace Modulo
{
	namespace Core
	{
		namespace Communication
		{
			namespace MessagePassing
			{
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
}