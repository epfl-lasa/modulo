

#ifndef MODULO_MOTION_STATECONVERSION_H_
#define MODULO_MOTION_STATECONVERSION_H_

#include "state_representation/Cartesian/CartesianState.hpp"
#include "state_representation/Joint/JointState.hpp"
#include "state_representation/DualQuaternion/DualQuaternionPose.hpp"
#include "state_representation/DualQuaternion/DualQuaternionTwist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "modulo_msgs/msg/jacobian_matrix.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "rcutils/logging_macros.h"
#include "rclcpp/time.hpp"

namespace ModuloCore
{
	namespace StateConversion
	{
		// update functions
		void update(StateRepresentation::CartesianState & state, const geometry_msgs::msg::Pose & msg);

		void update(StateRepresentation::CartesianState & state, const geometry_msgs::msg::PoseStamped & msg);

		void update(StateRepresentation::CartesianState & state, const geometry_msgs::msg::Transform & msg);

		void update(StateRepresentation::CartesianState & state, const geometry_msgs::msg::TransformStamped & msg);

		void update(StateRepresentation::CartesianState & state, const geometry_msgs::msg::Twist & msg);

		void update(StateRepresentation::CartesianState & state, const geometry_msgs::msg::TwistStamped & msg);

		void update(StateRepresentation::CartesianState & state, const geometry_msgs::msg::Accel & msg);

		void update(StateRepresentation::CartesianState & state, const geometry_msgs::msg::AccelStamped & msg);

		void update(StateRepresentation::CartesianState & state, const geometry_msgs::msg::Wrench & msg);

		void update(StateRepresentation::CartesianState & state, const geometry_msgs::msg::WrenchStamped & msg);

		void update(StateRepresentation::JointState & state, const sensor_msgs::msg::JointState & msg);

		void update(StateRepresentation::JointState & state, const modulo_msgs::msg::JacobianMatrix & msg);

		void update(StateRepresentation::DualQuaternionPose & state, const geometry_msgs::msg::Pose & msg);

		void update(StateRepresentation::DualQuaternionPose & state, const geometry_msgs::msg::PoseStamped & msg);

		void update(StateRepresentation::DualQuaternionTwist & state, const geometry_msgs::msg::Twist & msg);

		void update(StateRepresentation::DualQuaternionTwist & state, const geometry_msgs::msg::TwistStamped & msg);

		//extract functions
		void extract(geometry_msgs::msg::Quaternion & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time &);

		void extract(geometry_msgs::msg::Pose & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

		void extract(geometry_msgs::msg::PoseStamped & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

		void extract(geometry_msgs::msg::Transform & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

		void extract(geometry_msgs::msg::TransformStamped & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

		void extract(geometry_msgs::msg::Twist & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

		void extract(geometry_msgs::msg::TwistStamped & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

		void extract(geometry_msgs::msg::Accel & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

		void extract(geometry_msgs::msg::AccelStamped & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

		void extract(geometry_msgs::msg::Wrench & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

		void extract(geometry_msgs::msg::WrenchStamped & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

		void extract(sensor_msgs::msg::JointState & msg, const StateRepresentation::JointState & state, const rclcpp::Time & time);

		void extract(modulo_msgs::msg::JacobianMatrix & msg, const StateRepresentation::JointState & state, const rclcpp::Time & time);

		void extract(geometry_msgs::msg::Pose & msg, const StateRepresentation::DualQuaternionPose & state, const rclcpp::Time & time);

		void extract(geometry_msgs::msg::PoseStamped & msg, const StateRepresentation::DualQuaternionPose & state, const rclcpp::Time & time);

		void extract(geometry_msgs::msg::Twist & msg, const StateRepresentation::DualQuaternionTwist & state, const rclcpp::Time & time);

		void extract(geometry_msgs::msg::TwistStamped & msg, const StateRepresentation::DualQuaternionTwist & state, const rclcpp::Time & time);

		void extract(tf2_msgs::msg::TFMessage & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);
	}
}


#endif