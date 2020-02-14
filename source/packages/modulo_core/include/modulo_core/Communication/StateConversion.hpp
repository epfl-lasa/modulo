

#ifndef MODULO_MOTION_STATECONVERSION_H_
#define MODULO_MOTION_STATECONVERSION_H_

#include "state_representation/Space/Cartesian/CartesianState.hpp"
#include "state_representation/Space/Cartesian/CartesianPose.hpp"
#include "state_representation/Space/Cartesian/CartesianTwist.hpp"
#include "state_representation/Space/Cartesian/CartesianWrench.hpp"
#include "state_representation/Robot/JointState.hpp"
#include "state_representation/Robot/JacobianMatrix.hpp"
#include "state_representation/Space/DualQuaternion/DualQuaternionPose.hpp"
#include "state_representation/Space/DualQuaternion/DualQuaternionTwist.hpp"
#include "state_representation/Parameter/Parameter.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "modulo_msgs/msg/jacobian_matrix.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "nav_msgs/msg/odometry.hpp"
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
				/**
				 * @brief Convert a ROS geometry_msgs::msg::Pose to a CartesianState
				 * @param state The CartesianState to populate
				 * @param msg The ROS msg to read from
				 */
				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::Pose & msg);

				/**
				 * @brief Convert a ROS geometry_msgs::msg::PoseStamped to a CartesianState
				 * @param state The CartesianState to populate
				 * @param msg The ROS msg to read from
				 */
				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::PoseStamped & msg);

				/**
				 * @brief Convert a ROS geometry_msgs::msg::Transform to a CartesianState
				 * @param state The CartesianState to populate
				 * @param msg The ROS msg to read from
				 */
				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::Transform & msg);

				/**
				 * @brief Convert a ROS geometry_msgs::msg::TransformStamped to a CartesianState
				 * @param state The CartesianState to populate
				 * @param msg The ROS msg to read from
				 */
				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::TransformStamped & msg);

				/**
				 * @brief Convert a ROS geometry_msgs::msg::Twist to a CartesianState
				 * @param state The CartesianState to populate
				 * @param msg The ROS msg to read from
				 */
				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::Twist & msg);

				/**
				 * @brief Convert a ROS geometry_msgs::msg::TwistStamped to a CartesianState
				 * @param state The CartesianState to populate
				 * @param msg The ROS msg to read from
				 */
				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::TwistStamped & msg);

				/**
				 * @brief Convert a ROS geometry_msgs::msg::Accel to a CartesianState
				 * @param state The CartesianState to populate
				 * @param msg The ROS msg to read from
				 */
				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::Accel & msg);

				/**
				 * @brief Convert a ROS geometry_msgs::msg::AccelStamped to a CartesianState
				 * @param state The CartesianState to populate
				 * @param msg The ROS msg to read from
				 */
				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::AccelStamped & msg);

				/**
				 * @brief Convert a ROS geometry_msgs::msg::Wrench to a CartesianState
				 * @param state The CartesianState to populate
				 * @param msg The ROS msg to read from
				 */
				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::Wrench & msg);

				/**
				 * @brief Convert a ROS geometry_msgs::msg::WrenchStamped to a CartesianState
				 * @param state The CartesianState to populate
				 * @param msg The ROS msg to read from
				 */
				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::WrenchStamped & msg);

				/**
				 * @brief Convert a ROS geometry_msgs::msg::WrenchStamped to a CartesianState
				 * @param state The CartesianState to populate
				 * @param msg The ROS msg to read from
				 */
				void read_msg(StateRepresentation::CartesianState & state, const nav_msgs::msg::Odometry & msg);

				/**
				 * @brief Convert a ROS sensor_msgs::msg::JointState to a JointState
				 * @param state The JointState to populate
				 * @param msg The ROS msg to read from
				 */
				void read_msg(StateRepresentation::JointState & state, const sensor_msgs::msg::JointState & msg);

				/**
				 * @brief Convert a ROS modulo_msgs::msg::JacobianMatrix to a JacobianMatrix
				 * @param state The JacobianMatrix to populate
				 * @param msg The ROS msg to read from
				 */
				void read_msg(StateRepresentation::JacobianMatrix & state, const modulo_msgs::msg::JacobianMatrix & msg);

				/**
				 * @brief Convert a ROS geometry_msgs::msg::Pose to a DualQuaternionPose
				 * @param state The DualQuaternionPose to populate
				 * @param msg The ROS msg to read from
				 */
				void read_msg(StateRepresentation::DualQuaternionPose & state, const geometry_msgs::msg::Pose & msg);

				/**
				 * @brief Convert a ROS geometry_msgs::msg::PoseStamped to a DualQuaternionPose
				 * @param state The DualQuaternionPose to populate
				 * @param msg The ROS msg to read from
				 */
				void read_msg(StateRepresentation::DualQuaternionPose & state, const geometry_msgs::msg::PoseStamped & msg);

				/**
				 * @brief Convert a ROS geometry_msgs::msg::Twist to a DualQuaternionTwist
				 * @param state The DualQuaternionPose to populate
				 * @param msg The ROS msg to read from
				 */
				void read_msg(StateRepresentation::DualQuaternionTwist & state, const geometry_msgs::msg::Twist & msg);

				/**
				 * @brief Convert a ROS geometry_msgs::msg::TwistStamped to a DualQuaternionTwist
				 * @param state The DualQuaternionPose to populate
				 * @param msg The ROS msg to read from
				 */
				void read_msg(StateRepresentation::DualQuaternionTwist & state, const geometry_msgs::msg::TwistStamped & msg);

				/**
				 * @brief Convert a ROS std_msgs::msg::Float64 to a Parameter<double>
				 * @param state The Parameter<double> to populate
				 * @param msg The ROS msg to read from
				 */
				void read_msg(StateRepresentation::Parameter<double> & state, const std_msgs::msg::Float64 & msg);

				/**
				 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::Quaternion
				 * @param msg The ROS msg to populate
				 * @param state The state to read from
				 * @param time The time of the message
				 */
				void write_msg(geometry_msgs::msg::Quaternion & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

				/**
				 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::Pose
				 * @param msg The ROS msg to populate
				 * @param state The state to read from
				 * @param time The time of the message
				 */
				void write_msg(geometry_msgs::msg::Pose & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

				/**
				 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::PoseStamped
				 * @param msg The ROS msg to populate
				 * @param state The state to read from
				 * @param time The time of the message
				 */
				void write_msg(geometry_msgs::msg::PoseStamped & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

				/**
				 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::Transform
				 * @param msg The ROS msg to populate
				 * @param state The state to read from
				 * @param time The time of the message
				 */
				void write_msg(geometry_msgs::msg::Transform & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

				/**
				 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::TransformStamped
				 * @param msg The ROS msg to populate
				 * @param state The state to read from
				 * @param time The time of the message
				 */
				void write_msg(geometry_msgs::msg::TransformStamped & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

				/**
				 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::Twist
				 * @param msg The ROS msg to populate
				 * @param state The state to read from
				 * @param time The time of the message
				 */
				void write_msg(geometry_msgs::msg::Twist & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

				/**
				 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::TwistStamped
				 * @param msg The ROS msg to populate
				 * @param state The state to read from
				 * @param time The time of the message
				 */
				void write_msg(geometry_msgs::msg::TwistStamped & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

				/**
				 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::Accel
				 * @param msg The ROS msg to populate
				 * @param state The state to read from
				 * @param time The time of the message
				 */
				void write_msg(geometry_msgs::msg::Accel & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

				/**
				 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::AccelStamped
				 * @param msg The ROS msg to populate
				 * @param state The state to read from
				 * @param time The time of the message
				 */
				void write_msg(geometry_msgs::msg::AccelStamped & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

				/**
				 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::Wrench
				 * @param msg The ROS msg to populate
				 * @param state The state to read from
				 * @param time The time of the message
				 */
				void write_msg(geometry_msgs::msg::Wrench & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

				/**
				 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::WrenchStamped
				 * @param msg The ROS msg to populate
				 * @param state The state to read from
				 * @param time The time of the message
				 */
				void write_msg(geometry_msgs::msg::WrenchStamped & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

				/**
				 * @brief Convert a JointState to a ROS sensor_msgs::msg::JointState
				 * @param msg The ROS msg to populate
				 * @param state The state to read from
				 * @param time The time of the message
				 */
				void write_msg(sensor_msgs::msg::JointState & msg, const StateRepresentation::JointState & state, const rclcpp::Time & time);

				/**
				 * @brief Convert a JacobianMatrix to a ROS modulo_msgs::msg::JacobianMatrix
				 * @param msg The ROS msg to populate
				 * @param state The state to read from
				 * @param time The time of the message
				 */
				void write_msg(modulo_msgs::msg::JacobianMatrix & msg, const StateRepresentation::JacobianMatrix & state, const rclcpp::Time & time);

				/**
				 * @brief Convert a DualQuaternionPose to a ROS geometry_msgs::msg::Pose
				 * @param msg The ROS msg to populate
				 * @param state The state to read from
				 * @param time The time of the message
				 */
				void write_msg(geometry_msgs::msg::Pose & msg, const StateRepresentation::DualQuaternionPose & state, const rclcpp::Time & time);

				/**
				 * @brief Convert a DualQuaternionPose to a ROS geometry_msgs::msg::PoseStamped
				 * @param msg The ROS msg to populate
				 * @param state The state to read from
				 * @param time The time of the message
				 */
				void write_msg(geometry_msgs::msg::PoseStamped & msg, const StateRepresentation::DualQuaternionPose & state, const rclcpp::Time & time);

				/**
				 * @brief Convert a DualQuaternionPose to a ROS geometry_msgs::msg::Twist
				 * @param msg The ROS msg to populate
				 * @param state The state to read from
				 * @param time The time of the message
				 */
				void write_msg(geometry_msgs::msg::Twist & msg, const StateRepresentation::DualQuaternionTwist & state, const rclcpp::Time & time);

				/**
				 * @brief Convert a DualQuaternionPose to a ROS geometry_msgs::msg::TwistStamped
				 * @param msg The ROS msg to populate
				 * @param state The state to read from
				 * @param time The time of the message
				 */
				void write_msg(geometry_msgs::msg::TwistStamped & msg, const StateRepresentation::DualQuaternionTwist & state, const rclcpp::Time & time);

				/**
				 * @brief Convert a CartesianState to a ROS tf2_msgs::msg::TFMessage
				 * @param msg The ROS msg to populate
				 * @param state The state to read from
				 * @param time The time of the message
				 */
				void write_msg(tf2_msgs::msg::TFMessage & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time);

				/**
				 * @brief Convert a CartesianTwist to a ROS std_msgs::msg::Float64MultiArray
				 * @param msg The ROS msg to populate
				 * @param state The state to read from
				 * @param time The time of the message
				 */
				void write_msg(std_msgs::msg::Float64MultiArray & msg, const StateRepresentation::CartesianTwist & state, const rclcpp::Time & time);

				/**
				 * @brief Convert a Parameter<double> to a ROS std_msgs::msg::Float64
				 * @param msg The ROS msg to populate
				 * @param state The state to read from
				 * @param time The time of the message
				 */
				void write_msg(std_msgs::msg::Float64 & msg, const StateRepresentation::Parameter<double> & state, const rclcpp::Time & time);				
			}
		}
	}
}


#endif