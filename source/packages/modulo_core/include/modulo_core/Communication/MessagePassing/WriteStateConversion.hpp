

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
#include "state_representation/Parameters/Parameter.hpp"
#include "state_representation/Trajectories/Trajectory.hpp"
#include "state_representation/Geometry/Ellipsoid.hpp"
#include "state_representation/Exceptions/EmptyStateException.hpp"

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
					 * @brief Convert a JointState to a ROS trajectory_msgs::msg::JointTrajectoryPoint
					 * @param msg The ROS msg to populate
					 * @param state The state to read from
					 * @param time The time of the message
					 */
					void write_msg(trajectory_msgs::msg::JointTrajectoryPoint & msg, const StateRepresentation::JointState & state, const rclcpp::Time &);				

					/**
					 * @brief Convert a JointState to a ROS trajectory_msgs::msg::JointTrajectory
					 * @param msg The ROS msg to populate
					 * @param state The state to read from
					 * @param time The time of the message
					 */
					void write_msg(trajectory_msgs::msg::JointTrajectory & msg, const StateRepresentation::Trajectory<StateRepresentation::JointState> & state, const rclcpp::Time & time);

					/**
					 * @brief Convert a JointPositions to a ROS std_msgs::msg::Float64MultiArray
					 * @param msg The ROS msg to populate
					 * @param state The state to read from
					 * @param time The time of the message
					 */
					void write_msg(std_msgs::msg::Float64MultiArray & msg, const StateRepresentation::JointPositions & state, const rclcpp::Time & );

					/**
					 * @brief Convert a JointVelocities to a ROS std_msgs::msg::Float64MultiArray
					 * @param msg The ROS msg to populate
					 * @param state The state to read from
					 * @param time The time of the message
					 */
					void write_msg(std_msgs::msg::Float64MultiArray & msg, const StateRepresentation::JointVelocities & state, const rclcpp::Time & );
				
					/**
					 * @brief Convert a JointVelocities to a ROS std_msgs::msg::Float64MultiArray
					 * @param msg The ROS msg to populate
					 * @param state The state to read from
					 * @param time The time of the message
					 */
					void write_msg(std_msgs::msg::Float64MultiArray & msg, const StateRepresentation::Ellipsoid & state, const rclcpp::Time & );

					/**
					 * @brief Convert a Parameter<double> to a ROS std_msgs::msg::Float64
					 * @param msg The ROS msg to populate
					 * @param state The state to read from
					 * @param time The time of the message
					 */
					template <typename T, typename U>
					void write_msg(U & msg, const StateRepresentation::Parameter<T> & state, const rclcpp::Time &)
					{
						using namespace StateRepresentation::Exceptions;
						if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
						msg.data = state.get_value();
					}
				}
			}
		}
	}
}
