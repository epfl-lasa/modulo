#pragma once

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

#include <clproto.h>
#include <state_representation/parameters/Parameter.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/joint/JointState.hpp>

#include "modulo_new_core/EncodedState.hpp"

namespace modulo_new_core::translators {

/**
 * @brief Convert a ROS geometry_msgs::msg::Accel to a CartesianState
 * @param state The CartesianState to populate
 * @param msg The ROS msg to read from
 */
void read_msg(state_representation::CartesianState& state, const geometry_msgs::msg::Accel& msg);

/**
 * @brief Convert a ROS geometry_msgs::msg::AccelStamped to a CartesianState
 * @param state The CartesianState to populate
 * @param msg The ROS msg to read from
 */
void read_msg(state_representation::CartesianState& state, const geometry_msgs::msg::AccelStamped& msg);

/**
 * @brief Convert a ROS geometry_msgs::msg::Pose to a CartesianState
 * @param state The CartesianState to populate
 * @param msg The ROS msg to read from
 */
void read_msg(state_representation::CartesianState& state, const geometry_msgs::msg::Pose& msg);

/**
 * @brief Convert a ROS geometry_msgs::msg::PoseStamped to a CartesianState
 * @param state The CartesianState to populate
 * @param msg The ROS msg to read from
 */
void read_msg(state_representation::CartesianState& state, const geometry_msgs::msg::PoseStamped& msg);

/**
 * @brief Convert a ROS geometry_msgs::msg::Transform to a CartesianState
 * @param state The CartesianState to populate
 * @param msg The ROS msg to read from
 */
void read_msg(state_representation::CartesianState& state, const geometry_msgs::msg::Transform& msg);

/**
 * @brief Convert a ROS geometry_msgs::msg::TransformStamped to a CartesianState
 * @param state The CartesianState to populate
 * @param msg The ROS msg to read from
 */
void read_msg(state_representation::CartesianState& state, const geometry_msgs::msg::TransformStamped& msg);

/**
 * @brief Convert a ROS geometry_msgs::msg::Twist to a CartesianState
 * @param state The CartesianState to populate
 * @param msg The ROS msg to read from
 */
void read_msg(state_representation::CartesianState& state, const geometry_msgs::msg::Twist& msg);

/**
 * @brief Convert a ROS geometry_msgs::msg::TwistStamped to a CartesianState
 * @param state The CartesianState to populate
 * @param msg The ROS msg to read from
 */
void read_msg(state_representation::CartesianState& state, const geometry_msgs::msg::TwistStamped& msg);

/**
 * @brief Convert a ROS geometry_msgs::msg::Wrench to a CartesianState
 * @param state The CartesianState to populate
 * @param msg The ROS msg to read from
 */
void read_msg(state_representation::CartesianState& state, const geometry_msgs::msg::Wrench& msg);

/**
 * @brief Convert a ROS geometry_msgs::msg::WrenchStamped to a CartesianState
 * @param state The CartesianState to populate
 * @param msg The ROS msg to read from
 */
void read_msg(state_representation::CartesianState& state, const geometry_msgs::msg::WrenchStamped& msg);

/**
 * @brief Convert a ROS sensor_msgs::msg::JointState to a JointState
 * @param state The JointState to populate
 * @param msg The ROS msg to read from
 */
void read_msg(state_representation::JointState& state, const sensor_msgs::msg::JointState& msg);

/**
 * @brief Template function to convert a ROS std_msgs::msg::T to a Parameter<T>
 * @tparam T all types of parameters supported in ROS std messages
 * @tparam U all types of parameters supported in ROS std messages
 * @param state The Parameter<T> to populate
 * @param msg The ROS msg to read from
 */
template <typename T, typename U>
void read_msg(state_representation::Parameter<T>& state, const U& msg) {
  state.set_value(msg.data);
}

/**
 * @brief Convert a ROS std_msgs::msg::Bool to a boolean
 * @param state The state to populate
 * @param msg The ROS msg to read from
 */
void read_msg(bool& state, const std_msgs::msg::Bool& msg);

/**
 * @brief Convert a ROS std_msgs::msg::Float64 to a double
 * @param state The state to populate
 * @param msg The ROS msg to read from
 */
void read_msg(double& state, const std_msgs::msg::Float64& msg);

/**
 * @brief Convert a ROS std_msgs::msg::Float64MultiArray to a vector of double
 * @param state The state to populate
 * @param msg The ROS msg to read from
 */
void read_msg(std::vector<double>& state, const std_msgs::msg::Float64MultiArray& msg);

/**
 * @brief Convert a ROS std_msgs::msg::Int32 to an integer
 * @param state The state to populate
 * @param msg The ROS msg to read from
 */
void read_msg(int& state, const std_msgs::msg::Int32& msg);

/**
 * @brief Convert a ROS std_msgs::msg::String to a string
 * @param state The state to populate
 * @param msg The ROS msg to read from
 */
void read_msg(std::string& state, const std_msgs::msg::String& msg);

/**
 * @brief Convert a ROS std_msgs::msg::UInt8MultiArray message to a State using protobuf decoding
 * @tparam a state_representation::State type object
 * @param state The state to populate
 * @param msg The ROS msg to read from
 */
template <typename T>
inline void read_msg(T& state, const EncodedState& msg) {
  std::string tmp(msg.data.begin(), msg.data.end());
  state = clproto::decode<T>(tmp);
}
}// namespace modulo_new_core::translators