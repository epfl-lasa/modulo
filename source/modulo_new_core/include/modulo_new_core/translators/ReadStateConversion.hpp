#pragma once

#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <clproto.h>
#include <state_representation/parameters/Parameter.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/cartesian/CartesianTwist.hpp>
#include <state_representation/space/cartesian/CartesianWrench.hpp>
#include <state_representation/space/joint/JointState.hpp>

namespace modulo_new_core::translators {
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
 * @brief Convert a ROS geometry_msgs::msg::WrenchStamped to a CartesianState
 * @param state The CartesianState to populate
 * @param msg The ROS msg to read from
 */
void read_msg(state_representation::CartesianState& state, const nav_msgs::msg::Odometry& msg);

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
 * @brief Convert a ROS std_msgs::msg::UInt8MultiArray message to a State using protobuf decoding
 * @tparam a state_representation::State type object
 * @param state The state to populate
 * @param msg The ROS msg to read from
 */
template <typename T>
void read_msg(T& state, const std_msgs::msg::UInt8MultiArray& msg) {
  std::string tmp(msg.data.begin(), msg.data.end());
  state = clproto::decode<T>(tmp);
}
}// namespace modulo_new_core::translators