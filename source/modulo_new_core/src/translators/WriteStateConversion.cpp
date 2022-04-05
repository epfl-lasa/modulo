#include "modulo_new_core/translators/WriteStateConversion.hpp"

#include <state_representation/exceptions/EmptyStateException.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>

using namespace state_representation;

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

void write_msg(geometry_msgs::msg::Accel& msg, const CartesianState& state, const rclcpp::Time&) {
  if (state.is_empty()) {
    throw exceptions::EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  }
  write_vector3(msg.linear, state.get_linear_acceleration());
  write_vector3(msg.angular, state.get_angular_acceleration());
}

void write_msg(geometry_msgs::msg::AccelStamped& msg, const CartesianState& state, const rclcpp::Time& time) {
  write_msg(msg.accel, state, time);
  msg.header.stamp = time;
  msg.header.frame_id = state.get_reference_frame();
}

void write_msg(geometry_msgs::msg::Pose& msg, const CartesianState& state, const rclcpp::Time&) {
  if (state.is_empty()) {
    throw exceptions::EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  }
  write_point(msg.position, state.get_position());
  write_quaternion(msg.orientation, state.get_orientation());
}

void write_msg(geometry_msgs::msg::PoseStamped& msg, const CartesianState& state, const rclcpp::Time& time) {
  write_msg(msg.pose, state, time);
  msg.header.stamp = time;
  msg.header.frame_id = state.get_reference_frame();
}

void write_msg(geometry_msgs::msg::Transform& msg, const CartesianState& state, const rclcpp::Time&) {
  if (state.is_empty()) {
    throw exceptions::EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  }
  write_vector3(msg.translation, state.get_position());
  write_quaternion(msg.rotation, state.get_orientation());
}

void write_msg(geometry_msgs::msg::TransformStamped& msg, const CartesianState& state, const rclcpp::Time& time) {
  write_msg(msg.transform, state, time);
  msg.header.stamp = time;
  msg.header.frame_id = state.get_reference_frame();
  msg.child_frame_id = state.get_name();
}

void write_msg(geometry_msgs::msg::Twist& msg, const CartesianState& state, const rclcpp::Time&) {
  if (state.is_empty()) {
    throw exceptions::EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  }
  write_vector3(msg.linear, state.get_linear_velocity());
  write_vector3(msg.angular, state.get_angular_velocity());
}

void write_msg(geometry_msgs::msg::TwistStamped& msg, const CartesianState& state, const rclcpp::Time& time) {
  write_msg(msg.twist, state, time);
  msg.header.stamp = time;
  msg.header.frame_id = state.get_reference_frame();
}

void write_msg(geometry_msgs::msg::Wrench& msg, const CartesianState& state, const rclcpp::Time&) {
  if (state.is_empty()) {
    throw exceptions::EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  }
  write_vector3(msg.force, state.get_force());
  write_vector3(msg.torque, state.get_torque());
}

void write_msg(geometry_msgs::msg::WrenchStamped& msg, const CartesianState& state, const rclcpp::Time& time) {
  write_msg(msg.wrench, state, time);
  msg.header.stamp = time;
  msg.header.frame_id = state.get_reference_frame();
}

void write_msg(sensor_msgs::msg::JointState& msg, const JointState& state, const rclcpp::Time& time) {
  if (state.is_empty()) {
    throw exceptions::EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  }
  msg.header.stamp = time;
  msg.name = state.get_names();
  msg.position = std::vector<double>(state.get_positions().data(), state.get_positions().data() + state.get_positions().size());
  msg.velocity = std::vector<double>(state.get_velocities().data(), state.get_velocities().data() + state.get_velocities().size());
  msg.effort = std::vector<double>(state.get_torques().data(), state.get_torques().data() + state.get_torques().size());
}

void write_msg(tf2_msgs::msg::TFMessage& msg, const CartesianState& state, const rclcpp::Time& time) {
  if (state.is_empty()) {
    throw exceptions::EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  }
  geometry_msgs::msg::TransformStamped transform;
  write_msg(transform, state, time);
  msg.transforms.push_back(transform);
}

template <typename U, typename T>
void write_msg(U& msg, const Parameter<T>& state, const rclcpp::Time&) {
  if (state.is_empty()) {
    throw exceptions::EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  }
  msg.data = state.get_value();
}

template void write_msg<std_msgs::msg::Float64, double>(std_msgs::msg::Float64& msg, const Parameter<double>& state, const rclcpp::Time&);

template void write_msg<std_msgs::msg::Float64MultiArray, std::vector<double>>(std_msgs::msg::Float64MultiArray& msg, const Parameter<std::vector<double>>& state, const rclcpp::Time&);

template void write_msg<std_msgs::msg::Bool, bool>(std_msgs::msg::Bool& msg, const Parameter<bool>& state, const rclcpp::Time&);

template void write_msg<std_msgs::msg::String, std::string>(std_msgs::msg::String& msg, const Parameter<std::string>& state, const rclcpp::Time&);

template <>
void write_msg(geometry_msgs::msg::Transform& msg, const Parameter<CartesianPose>& state, const rclcpp::Time& time) {
  write_msg(msg, state.get_value(), time);
}

template <>
void write_msg(geometry_msgs::msg::TransformStamped& msg, const Parameter<CartesianPose>& state, const rclcpp::Time& time) {
  write_msg(msg, state.get_value(), time);
}

template <>
void write_msg(tf2_msgs::msg::TFMessage& msg, const Parameter<CartesianPose>& state, const rclcpp::Time& time) {
  write_msg(msg, state.get_value(), time);
}

void write_msg(std_msgs::msg::Bool& msg, const bool& state, const rclcpp::Time&) {
  msg.data = state;
}

void write_msg(std_msgs::msg::Float64& msg, const double& state, const rclcpp::Time&) {
  msg.data = state;
}

void write_msg(std_msgs::msg::Float64MultiArray& msg, const std::vector<double>& state, const rclcpp::Time&) {
  msg.data = state;
}

void write_msg(std_msgs::msg::Int32& msg, const int& state, const rclcpp::Time&) {
  msg.data = state;
}

void write_msg(std_msgs::msg::String& msg, const std::string& state, const rclcpp::Time&) {
  msg.data = state;
}
}// namespace modulo_new_core::translators