

#ifndef MODULO_MOTION_STATECONVERSION_H_
#define MODULO_MOTION_STATECONVERSION_H_

#include "state_representation/Cartesian/CartesianState.hpp"
#include "state_representation/Joint/JointState.hpp"
#include "state_representation/Robot/JacobianMatrix.hpp"
#include "state_representation/DualQuaternion/DualQuaternionPose.hpp"
#include "state_representation/DualQuaternion/DualQuaternionTwist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "modulo_msgs/msg/jacobian_matrix.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
// #include "std_msgs/msg/float32.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "rcutils/logging_macros.h"
#include "rclcpp/time.hpp"


namespace Modulo
{
	namespace Core
	{
		namespace Communication
		{
			namespace StateConversion
			{
				// read_msg functions
				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::Pose & msg);

				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::PoseStamped & msg);

				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::Transform & msg);

				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::TransformStamped & msg);

				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::Twist & msg);

				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::TwistStamped & msg);

				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::Accel & msg);

				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::AccelStamped & msg);

				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::Wrench & msg);

				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::WrenchStamped & msg);

				void read_msg(StateRepresentation::JointState & state, const sensor_msgs::msg::JointState & msg);

				void read_msg(StateRepresentation::JacobianMatrix & state, const modulo_msgs::msg::JacobianMatrix & msg);

				void read_msg(StateRepresentation::DualQuaternionPose & state, const geometry_msgs::msg::Pose & msg);

				void read_msg(StateRepresentation::DualQuaternionPose & state, const geometry_msgs::msg::PoseStamped & msg);

				void read_msg(StateRepresentation::DualQuaternionTwist & state, const geometry_msgs::msg::Twist & msg);

				void read_msg(StateRepresentation::DualQuaternionTwist & state, const geometry_msgs::msg::TwistStamped & msg);

				//write_msg functions
				void write_msg(geometry_msgs::msg::Quaternion & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time &);

				void write_msg(geometry_msgs::msg::Pose & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

				void write_msg(geometry_msgs::msg::PoseStamped & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

				void write_msg(geometry_msgs::msg::Transform & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

				void write_msg(geometry_msgs::msg::TransformStamped & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

				void write_msg(geometry_msgs::msg::Twist & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

				void write_msg(geometry_msgs::msg::TwistStamped & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

				void write_msg(geometry_msgs::msg::Accel & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

				void write_msg(geometry_msgs::msg::AccelStamped & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

				void write_msg(geometry_msgs::msg::Wrench & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

				void write_msg(geometry_msgs::msg::WrenchStamped & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

				void write_msg(sensor_msgs::msg::JointState & msg, const StateRepresentation::JointState & state, const rclcpp::Time & time);

				void write_msg(modulo_msgs::msg::JacobianMatrix & msg, const StateRepresentation::JacobianMatrix & state, const rclcpp::Time & time);

				void write_msg(geometry_msgs::msg::Pose & msg, const StateRepresentation::DualQuaternionPose & state, const rclcpp::Time & time);

				void write_msg(geometry_msgs::msg::PoseStamped & msg, const StateRepresentation::DualQuaternionPose & state, const rclcpp::Time & time);

				void write_msg(geometry_msgs::msg::Twist & msg, const StateRepresentation::DualQuaternionTwist & state, const rclcpp::Time & time);

				void write_msg(geometry_msgs::msg::TwistStamped & msg, const StateRepresentation::DualQuaternionTwist & state, const rclcpp::Time & time);

				void write_msg(tf2_msgs::msg::TFMessage & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

				void write_msg(std_msgs::msg::Float64MultiArray & msg, const StateRepresentation::CartesianTwist & state, const rclcpp::Time & time);

			}
		}
	}
}


#endif