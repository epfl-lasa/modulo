#include "modulo_new_core/translators/WriteStateConversion.hpp"

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

using namespace state_representation::exceptions;

namespace modulo_new_core::translators {

static void write_point(geometry_msgs::msg::Point& msg, const Eigen::Vector3d& vector) {
  msg.x = vector.x();
  msg.y = vector.y();
  msg.z = vector.z();
}

static void write_vector3(geometry_msgs::msg::Vector3& msg, const Eigen::Vector3d& vector) {
  msg.x = vector.x();
  msg.y = vector.y();
  msg.z = vector.z();
}

static void write_quaternion(geometry_msgs::msg::Quaternion& msg, const Eigen::Quaterniond& quat) {
  msg.w = quat.w();
  msg.x = quat.x();
  msg.y = quat.y();
  msg.z = quat.z();
}

void write_msg(geometry_msgs::msg::Point& msg, const state_representation::CartesianState& state, const rclcpp::Time&) {
  if (state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  write_point(msg, state.get_position());
}

void write_msg(geometry_msgs::msg::Vector3& msg, const state_representation::CartesianState& state, const rclcpp::Time&) {
  if (state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  write_vector3(msg, state.get_position());
}

void write_msg(geometry_msgs::msg::Quaternion& msg, const state_representation::CartesianState& state, const rclcpp::Time&) {
  if (state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  write_quaternion(msg, state.get_orientation());
}

void write_msg(geometry_msgs::msg::Accel& msg, const state_representation::CartesianState& state, const rclcpp::Time&) {
  if (state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  write_vector3(msg.linear, state.get_linear_acceleration());
  write_vector3(msg.angular, state.get_angular_acceleration());
}

void write_msg(geometry_msgs::msg::AccelStamped& msg, const state_representation::CartesianState& state, const rclcpp::Time& time) {
  msg.header.stamp = time;
  msg.header.frame_id = state.get_reference_frame();
  write_msg(msg.accel, state, time);
}


void write_msg(geometry_msgs::msg::Pose& msg, const state_representation::CartesianState& state, const rclcpp::Time&) {
  if (state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  write_point(msg.position, state.get_position());
  write_quaternion(msg.orientation, state.get_orientation());
}

void write_msg(geometry_msgs::msg::PoseStamped& msg, const state_representation::CartesianState& state, const rclcpp::Time& time) {
  msg.header.stamp = time;
  msg.header.frame_id = state.get_reference_frame();
  write_msg(msg.pose, state, time);
}

void write_msg(geometry_msgs::msg::Transform& msg, const state_representation::CartesianState& state, const rclcpp::Time&) {
  if (state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  write_vector3(msg.translation, state.get_position());
  write_quaternion(msg.rotation, state.get_orientation());
}

void write_msg(geometry_msgs::msg::TransformStamped& msg, const state_representation::CartesianState& state, const rclcpp::Time& time) {
  msg.header.stamp = time;
  msg.header.frame_id = state.get_reference_frame();
  msg.child_frame_id = state.get_name();
  write_msg(msg.transform, state, time);
}

void write_msg(geometry_msgs::msg::Twist& msg, const state_representation::CartesianState& state, const rclcpp::Time&) {
  if (state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  write_vector3(msg.linear, state.get_linear_velocity());
  write_vector3(msg.angular, state.get_angular_velocity());
}

void write_msg(geometry_msgs::msg::TwistStamped& msg, const state_representation::CartesianState& state, const rclcpp::Time& time) {
  msg.header.stamp = time;
  msg.header.frame_id = state.get_reference_frame();
  write_msg(msg.twist, state, time);
}

void write_msg(geometry_msgs::msg::Wrench& msg, const state_representation::CartesianState& state, const rclcpp::Time&) {
  if (state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  write_vector3(msg.force, state.get_force());
  write_vector3(msg.torque, state.get_torque());
}

void write_msg(geometry_msgs::msg::WrenchStamped& msg, const state_representation::CartesianState& state, const rclcpp::Time& time) {
  msg.header.stamp = time;
  msg.header.frame_id = state.get_reference_frame();
  write_msg(msg.wrench, state, time);
}

void write_msg(sensor_msgs::msg::JointState& msg, const state_representation::JointState& state, const rclcpp::Time& time) {
  if (state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  msg.header.stamp = time;
  msg.name = state.get_names();
  msg.position = std::vector<double>(state.get_positions().data(), state.get_positions().data() + state.get_positions().size());
  msg.velocity = std::vector<double>(state.get_velocities().data(), state.get_velocities().data() + state.get_velocities().size());
  msg.effort = std::vector<double>(state.get_torques().data(), state.get_torques().data() + state.get_torques().size());
}

void write_msg(tf2_msgs::msg::TFMessage& msg, const state_representation::CartesianState& state, const rclcpp::Time& time) {
  if (state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  geometry_msgs::msg::TransformStamped transform;
  write_msg(transform, state, time);
  msg.transforms.push_back(transform);
}

template <typename U, typename T>
void write_msg(U& msg, const state_representation::Parameter<T>& state, const rclcpp::Time&) {
  using namespace state_representation::exceptions;
  if (state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  msg.data = state.get_value();
}

template void write_msg<std_msgs::msg::Float64, double>(std_msgs::msg::Float64& msg, const state_representation::Parameter<double>& state, const rclcpp::Time&);

template void write_msg<std_msgs::msg::Float64MultiArray, std::vector<double>>(std_msgs::msg::Float64MultiArray& msg, const state_representation::Parameter<std::vector<double>>& state, const rclcpp::Time&);

template void write_msg<std_msgs::msg::Bool, bool>(std_msgs::msg::Bool& msg, const state_representation::Parameter<bool>& state, const rclcpp::Time&);

template void write_msg<std_msgs::msg::String, std::string>(std_msgs::msg::String& msg, const state_representation::Parameter<std::string>& state, const rclcpp::Time&);

template <>
void write_msg(geometry_msgs::msg::Transform& msg, const state_representation::Parameter<state_representation::CartesianPose>& state, const rclcpp::Time& time) {
  write_msg(msg, state.get_value(), time);
}

template <>
void write_msg(geometry_msgs::msg::TransformStamped& msg, const state_representation::Parameter<state_representation::CartesianPose>& state, const rclcpp::Time& time) {
  write_msg(msg, state.get_value(), time);
}

template <>
void write_msg(tf2_msgs::msg::TFMessage& msg, const state_representation::Parameter<state_representation::CartesianPose>& state, const rclcpp::Time& time) {
  write_msg(msg, state.get_value(), time);
}
}// namespace modulo_new_core::translators