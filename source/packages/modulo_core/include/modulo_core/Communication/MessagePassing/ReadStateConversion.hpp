#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <modulo_msgs/msg/jacobian_matrix.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcutils/logging_macros.h>
#include <rclcpp/time.hpp>
#include "state_representation/Space/Cartesian/CartesianState.hpp"
#include "state_representation/Space/Cartesian/CartesianPose.hpp"
#include "state_representation/Space/Cartesian/CartesianTwist.hpp"
#include "state_representation/Space/Cartesian/CartesianWrench.hpp"
#include "state_representation/Robot/JointState.hpp"
#include "state_representation/Robot/JacobianMatrix.hpp"
#include "state_representation/Space/DualQuaternion/DualQuaternionPose.hpp"
#include "state_representation/Space/DualQuaternion/DualQuaternionTwist.hpp"
#include "state_representation/Parameter/Parameter.hpp"
#include "state_representation/Trajectories/Trajectory.hpp"


namespace Modulo
{
	namespace Core
	{
		namespace Communication
		{
			namespace MessagePassing
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
					 * @brief Template function to convert a ROS std_msgs::msg::T to a Parameter<T>
					 * @param state The Parameter<T> to populate
					 * @param msg The ROS msg to read from
					 * @tparam T all types of parameters supported in ROS std messages
					 * @tparam U all types of parameters supported in ROS std messages
					 */
					template <typename T, typename U>
					void read_msg(StateRepresentation::Parameter<T> & state, const U & msg)
					{
						state.set_value(msg.data);
					}
				}
			}
		}
	}
}