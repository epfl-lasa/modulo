#pragma once

#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <clproto.h>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/joint/JointState.hpp>
#include <state_representation/parameters/Parameter.hpp>

#include "modulo_new_core/EncodedState.hpp"

namespace modulo_new_core::translators {

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::Accel
 * @param msg The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(geometry_msgs::msg::Accel& msg, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::AccelStamped
 * @param msg The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(geometry_msgs::msg::AccelStamped& msg, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::Pose
 * @param msg The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(geometry_msgs::msg::Pose& msg, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::PoseStamped
 * @param msg The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(geometry_msgs::msg::PoseStamped& msg, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::Transform
 * @param msg The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(geometry_msgs::msg::Transform& msg, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::TransformStamped
 * @param msg The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(geometry_msgs::msg::TransformStamped& msg, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::Twist
 * @param msg The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(geometry_msgs::msg::Twist& msg, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::TwistStamped
 * @param msg The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(geometry_msgs::msg::TwistStamped& msg, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::Wrench
 * @param msg The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(geometry_msgs::msg::Wrench& msg, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::WrenchStamped
 * @param msg The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(geometry_msgs::msg::WrenchStamped& msg, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a JointState to a ROS sensor_msgs::msg::JointState
 * @param msg The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(sensor_msgs::msg::JointState& msg, const state_representation::JointState& state, const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS tf2_msgs::msg::TFMessage
 * @param msg The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(tf2_msgs::msg::TFMessage& msg, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a Parameter<T> to a ROS equivalent representation
 * @tparam T all types of parameters supported in ROS std messages
 * @tparam U all types of parameters supported in ROS std messages
 * @param msg The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 */
template <typename U, typename T>
void write_msg(U& msg, const state_representation::Parameter<T>& state, const rclcpp::Time&);

/**
 * @brief Convert a boolean to a ROS std_msgs::msg::Bool
 * @param msg The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(std_msgs::msg::Bool& msg, const bool& state, const rclcpp::Time& time);

/**
 * @brief Convert a double to a ROS std_msgs::msg::Float64
 * @param msg The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(std_msgs::msg::Float64& msg, const double& state, const rclcpp::Time& time);

/**
 * @brief Convert a vector of double to a ROS std_msgs::msg::Float64MultiArray
 * @param msg The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(std_msgs::msg::Float64MultiArray& msg, const std::vector<double>& state, const rclcpp::Time& time);

/**
 * @brief Convert an integer to a ROS std_msgs::msg::Int32
 * @param msg The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(std_msgs::msg::Int32& msg, const int& state, const rclcpp::Time& time);

/**
 * @brief Convert a string to a ROS std_msgs::msg::String
 * @param msg The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_msg(std_msgs::msg::String& msg, const std::string& state, const rclcpp::Time& time);

/**
 * @brief Convert a state to a ROS std_msgs::msg::UInt8MultiArray message using protobuf encoding
 * @tparam a state_representation::State type object
 * @param msg The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 */
template<typename T>
inline void write_msg(EncodedState& msg, const T& state, const rclcpp::Time&) {
  std::string tmp = clproto::encode<T>(state);
  msg.data = std::vector<unsigned char>(tmp.begin(), tmp.end());
}
}// namespace modulo_new_core::translators