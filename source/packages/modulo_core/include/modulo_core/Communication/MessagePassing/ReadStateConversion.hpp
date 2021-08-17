#pragma once

#include <clproto.h>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <modulo_msgs/msg/jacobian.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <state_representation/parameters/Parameter.hpp>
#include <state_representation/robot/Jacobian.hpp>
#include <state_representation/robot/JointState.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/cartesian/CartesianTwist.hpp>
#include <state_representation/space/cartesian/CartesianWrench.hpp>
#include <state_representation/space/dual_quaternion/DualQuaternionPose.hpp>
#include <state_representation/space/dual_quaternion/DualQuaternionTwist.hpp>
#include <state_representation/trajectories/Trajectory.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "modulo_core/Communication/EncodedState.hpp"

namespace modulo::core::communication::state_conversion {
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
 * @brief Convert a ROS modulo_msgs::msg::Jacobian to a Jacobian
 * @param state The Jacobian to populate
 * @param msg The ROS msg to read from
 */
void read_msg(state_representation::Jacobian& state, const modulo_msgs::msg::Jacobian& msg);

/**
 * @brief Convert a ROS geometry_msgs::msg::Pose to a DualQuaternionPose
 * @param state The DualQuaternionPose to populate
 * @param msg The ROS msg to read from
 */
void read_msg(state_representation::DualQuaternionPose& state, const geometry_msgs::msg::Pose& msg);

/**
 * @brief Convert a ROS geometry_msgs::msg::PoseStamped to a DualQuaternionPose
 * @param state The DualQuaternionPose to populate
 * @param msg The ROS msg to read from
 */
void read_msg(state_representation::DualQuaternionPose& state, const geometry_msgs::msg::PoseStamped& msg);

/**
 * @brief Convert a ROS geometry_msgs::msg::Twist to a DualQuaternionTwist
 * @param state The DualQuaternionPose to populate
 * @param msg The ROS msg to read from
 */
void read_msg(state_representation::DualQuaternionTwist& state, const geometry_msgs::msg::Twist& msg);

/**
 * @brief Convert a ROS geometry_msgs::msg::TwistStamped to a DualQuaternionTwist
 * @param state The DualQuaternionPose to populate
 * @param msg The ROS msg to read from
 */
void read_msg(state_representation::DualQuaternionTwist& state, const geometry_msgs::msg::TwistStamped& msg);

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
 * @brief Convert an EncodedState type message to a State using protobuf decoding
 * @tparam a state_representation::State type object
 * @param state The state to populate
 * @param msg The ROS msg to read from
 */
template <typename T>
void read_msg(T& state, const EncodedState& msg) {
  std::string tmp(msg.data.begin(), msg.data.end());
  state = clproto::decode<T>(tmp);
}
}// namespace modulo::core::communication::state_conversion