#pragma once

#include <clproto.h>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <state_representation/exceptions/EmptyStateException.hpp>
#include <state_representation/geometry/Ellipsoid.hpp>
#include <state_representation/parameters/Parameter.hpp>
#include <state_representation/robot/JointState.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/cartesian/CartesianTwist.hpp>
#include <state_representation/space/cartesian/CartesianWrench.hpp>
#include <state_representation/space/dual_quaternion/DualQuaternionPose.hpp>
#include <state_representation/space/dual_quaternion/DualQuaternionTwist.hpp>
#include <state_representation/trajectories/Trajectory.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "modulo_core/Communication/EncodedState.hpp"

namespace modulo::core::communication::state_conversion {
/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::Quaternion
 * @param msg The ROS msg to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(geometry_msgs::msg::Quaternion& msg, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::Pose
 * @param msg The ROS msg to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(geometry_msgs::msg::Pose& msg, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::PoseStamped
 * @param msg The ROS msg to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(geometry_msgs::msg::PoseStamped& msg, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::Transform
 * @param msg The ROS msg to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(geometry_msgs::msg::Transform& msg, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::TransformStamped
 * @param msg The ROS msg to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(geometry_msgs::msg::TransformStamped& msg, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::Twist
 * @param msg The ROS msg to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(geometry_msgs::msg::Twist& msg, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::TwistStamped
 * @param msg The ROS msg to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(geometry_msgs::msg::TwistStamped& msg, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::Accel
 * @param msg The ROS msg to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(geometry_msgs::msg::Accel& msg, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::AccelStamped
 * @param msg The ROS msg to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(geometry_msgs::msg::AccelStamped& msg, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::Wrench
 * @param msg The ROS msg to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(geometry_msgs::msg::Wrench& msg, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::WrenchStamped
 * @param msg The ROS msg to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(geometry_msgs::msg::WrenchStamped& msg, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a JointState to a ROS sensor_msgs::msg::JointState
 * @param msg The ROS msg to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(sensor_msgs::msg::JointState& msg, const state_representation::JointState& state, const rclcpp::Time& time);

/**
 * @brief Convert a DualQuaternionPose to a ROS geometry_msgs::msg::Pose
 * @param msg The ROS msg to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(geometry_msgs::msg::Pose& msg, const state_representation::DualQuaternionPose& state, const rclcpp::Time& time);

/**
 * @brief Convert a DualQuaternionPose to a ROS geometry_msgs::msg::PoseStamped
 * @param msg The ROS msg to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(geometry_msgs::msg::PoseStamped& msg, const state_representation::DualQuaternionPose& state, const rclcpp::Time& time);

/**
 * @brief Convert a DualQuaternionPose to a ROS geometry_msgs::msg::Twist
 * @param msg The ROS msg to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(geometry_msgs::msg::Twist& msg, const state_representation::DualQuaternionTwist& state, const rclcpp::Time& time);

/**
 * @brief Convert a DualQuaternionPose to a ROS geometry_msgs::msg::TwistStamped
 * @param msg The ROS msg to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(geometry_msgs::msg::TwistStamped& msg, const state_representation::DualQuaternionTwist& state, const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS tf2_msgs::msg::TFMessage
 * @param msg The ROS msg to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(tf2_msgs::msg::TFMessage& msg, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a JointState to a ROS trajectory_msgs::msg::JointTrajectoryPoint
 * @param msg The ROS msg to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(trajectory_msgs::msg::JointTrajectoryPoint& msg, const state_representation::JointState& state, const rclcpp::Time&);

/**
 * @brief Convert a JointState to a ROS trajectory_msgs::msg::JointTrajectory
 * @param msg The ROS msg to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(trajectory_msgs::msg::JointTrajectory& msg, const state_representation::Trajectory<state_representation::JointState>& state, const rclcpp::Time& time);

/**
 * @brief Convert a Parameter<T> to a ROS equivalent representation
 * @tparam T all types of parameters supported in ROS std messages
 * @tparam U all types of parameters supported in ROS std messages
 * @param msg The ROS msg to populate
 * @param state The state to read from
 * @param time The time of the message
 */
template <typename U, typename T>
void write_msg(U& msg, const state_representation::Parameter<T>& state, const rclcpp::Time&);

/**
 * @brief Convert a state to a ROS std_msgs::msg::Float64MultiArray
 * @tparam a state_representation::State type object
 * @param msg The ROS msg to populate
 * @param state The state to read from
 * @param time The time of the message
 */
template <typename T>
void write_msg(std_msgs::msg::Float64MultiArray& msg, const T& state, const rclcpp::Time&) {
  if (state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  msg.data = state.to_std_vector();
}

/**
 * @brief Convert a state to an EncodedState type message using protobuf encoding
 * @tparam a state_representation::State type object
 * @param msg The ROS msg to populate
 * @param state The state to read from
 * @param time The time of the message
 */
template <typename T>
void write_msg(EncodedState& msg, const T& state, const rclcpp::Time&) {
  std::string tmp = clproto::encode<T>(state);
  msg.data = std::vector<unsigned char>(tmp.begin(), tmp.end());
}
}// namespace modulo::core::communication::state_conversion
