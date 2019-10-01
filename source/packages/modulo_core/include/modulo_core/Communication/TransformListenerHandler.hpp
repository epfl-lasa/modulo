/**
 * @class TransformListenerHandler
 * @brief Class to define a transform listener
 * @author Baptiste Busch
 * @date 2019/06/14
 *
 */

#ifndef MODULO_COMMUNICATION_TRANSFORMLISTENERHANDLER_H_
#define MODULO_COMMUNICATION_TRANSFORMLISTENERHANDLER_H_

#include "modulo_core/Communication/CommunicationHandler.hpp"
#include "tf2_ros/transform_listener.h"

namespace Modulo
{
	namespace Core
	{
		namespace Communication
		{
			class TransformListenerHandler: public CommunicationHandler
			{
			private:
				tf2_ros::Buffer buffer_;
				std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

			public:
				explicit TransformListenerHandler(const std::shared_ptr<StateRepresentation::CartesianPose>& recipient, const std::chrono::milliseconds& timeout, const std::shared_ptr<rclcpp::Clock>& clock, const std::shared_ptr<std::mutex>& mutex):
				CommunicationHandler("tf_listener", "tf_listener", recipient, timeout, clock, mutex), buffer_(clock) 
				{
					this->tf_listener_ = std::make_unique<tf2_ros::TransformListener>(buffer_);
				}

				explicit TransformListenerHandler(const std::chrono::milliseconds& timeout, const std::shared_ptr<rclcpp::Clock>& clock, std::shared_ptr<std::mutex>& mutex):
				CommunicationHandler("tf_listener", "tf_listener", std::make_shared<StateRepresentation::CartesianPose>(), timeout, clock, mutex), buffer_(clock) 
				{
					this->tf_listener_ = std::make_unique<tf2_ros::TransformListener>(buffer_);
				}
				const StateRepresentation::CartesianPose lookup_transform(const std::string& frame_name, const std::string& reference_frame) const
				{
					geometry_msgs::msg::TransformStamped transformStamped;
					StateRepresentation::CartesianPose result(frame_name, reference_frame);
					transformStamped = this->buffer_.lookupTransform(reference_frame, frame_name, tf2::TimePoint(std::chrono::milliseconds(0)), tf2::Duration(this->get_timeout()));
	    			StateConversion::read_msg(result, transformStamped);
	      			return result;
				}
		    };
		}
	}
}
#endif