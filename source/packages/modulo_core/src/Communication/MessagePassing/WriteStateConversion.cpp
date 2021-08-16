#include "modulo_core/Communication/MessagePassing/WriteStateConversion.hpp"

#include <clproto.h>
#include <state_representation/exceptions/IncompatibleReferenceFramesException.hpp>

using namespace state_representation::exceptions;

namespace modulo::core::communication::state_conversion {
void write_msg(geometry_msgs::msg::Quaternion& msg, const state_representation::CartesianState& state, const rclcpp::Time&) {
  if (state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  // orientation
  msg.w = state.get_orientation().w();
  msg.x = state.get_orientation().x();
  msg.y = state.get_orientation().y();
  msg.z = state.get_orientation().z();
}

void write_msg(geometry_msgs::msg::Pose& msg, const state_representation::CartesianState& state, const rclcpp::Time&) {
  if (state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  // position
  msg.position.x = state.get_position()(0);
  msg.position.y = state.get_position()(1);
  msg.position.z = state.get_position()(2);
  // orientation
  msg.orientation.w = state.get_orientation().w();
  msg.orientation.x = state.get_orientation().x();
  msg.orientation.y = state.get_orientation().y();
  msg.orientation.z = state.get_orientation().z();
}

void write_msg(geometry_msgs::msg::PoseStamped& msg, const state_representation::CartesianState& state, const rclcpp::Time& time) {
  msg.header.stamp = time;
  msg.header.frame_id = state.get_reference_frame();
  write_msg(msg.pose, state, time);
}

void write_msg(geometry_msgs::msg::Transform& msg, const state_representation::CartesianState& state, const rclcpp::Time&) {
  if (state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  // position
  msg.translation.x = state.get_position()(0);
  msg.translation.y = state.get_position()(1);
  msg.translation.z = state.get_position()(2);
  // orientation
  msg.rotation.w = state.get_orientation().w();
  msg.rotation.x = state.get_orientation().x();
  msg.rotation.y = state.get_orientation().y();
  msg.rotation.z = state.get_orientation().z();
}

void write_msg(geometry_msgs::msg::TransformStamped& msg, const state_representation::CartesianState& state, const rclcpp::Time& time) {
  msg.header.stamp = time;
  msg.header.frame_id = state.get_reference_frame();
  msg.child_frame_id = state.get_name();
  write_msg(msg.transform, state, time);
}

void write_msg(geometry_msgs::msg::Twist& msg, const state_representation::CartesianState& state, const rclcpp::Time&) {
  if (state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  // linear velocity
  msg.linear.x = state.get_linear_velocity()(0);
  msg.linear.y = state.get_linear_velocity()(1);
  msg.linear.z = state.get_linear_velocity()(2);
  // angular velocity
  msg.angular.x = state.get_angular_velocity()(0);
  msg.angular.y = state.get_angular_velocity()(1);
  msg.angular.z = state.get_angular_velocity()(2);
}

void write_msg(geometry_msgs::msg::TwistStamped& msg, const state_representation::CartesianState& state, const rclcpp::Time& time) {
  msg.header.stamp = time;
  msg.header.frame_id = state.get_reference_frame();
  write_msg(msg.twist, state, time);
}

void write_msg(geometry_msgs::msg::Accel& msg, const state_representation::CartesianState& state, const rclcpp::Time&) {
  if (state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  // linear acceleration
  msg.linear.x = state.get_linear_acceleration()(0);
  msg.linear.y = state.get_linear_acceleration()(1);
  msg.linear.z = state.get_linear_acceleration()(2);
  // angular acceleration
  msg.angular.x = state.get_angular_acceleration()(0);
  msg.angular.y = state.get_angular_acceleration()(1);
  msg.angular.z = state.get_angular_acceleration()(2);
}

void write_msg(geometry_msgs::msg::AccelStamped& msg, const state_representation::CartesianState& state, const rclcpp::Time& time) {
  msg.header.stamp = time;
  msg.header.frame_id = state.get_reference_frame();
  write_msg(msg.accel, state, time);
}

void write_msg(geometry_msgs::msg::Wrench& msg, const state_representation::CartesianState& state, const rclcpp::Time&) {
  if (state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  // force
  msg.force.x = state.get_force()(0);
  msg.force.y = state.get_force()(1);
  msg.force.z = state.get_force()(2);
  // torque
  msg.torque.x = state.get_torque()(0);
  msg.torque.y = state.get_torque()(1);
  msg.torque.z = state.get_torque()(2);
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

void write_msg(modulo_msgs::msg::Jacobian& msg, const state_representation::Jacobian& state, const rclcpp::Time& time) {
  if (state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  msg.header.stamp = time;
  msg.nb_dimensions = state.rows();
  msg.nb_joints = state.cols();
  msg.joint_names = state.get_joint_names();
  msg.data = std::vector<double>(state.data().data(), state.data().data() + state.data().size());
}

void write_msg(geometry_msgs::msg::Pose& msg, const state_representation::DualQuaternionPose& state, const rclcpp::Time&) {
  if (state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  // position
  msg.position.x = state.get_position()(0);
  msg.position.y = state.get_position()(1);
  msg.position.z = state.get_position()(2);
  // orientation
  msg.orientation.w = state.get_orientation().w();
  msg.orientation.x = state.get_orientation().x();
  msg.orientation.y = state.get_orientation().y();
  msg.orientation.z = state.get_orientation().z();
}

void write_msg(geometry_msgs::msg::PoseStamped& msg, const state_representation::DualQuaternionPose& state, const rclcpp::Time& time) {
  msg.header.stamp = time;
  msg.header.frame_id = state.get_reference_frame();
  write_msg(msg.pose, state, time);
}

void write_msg(geometry_msgs::msg::Twist& msg, const state_representation::DualQuaternionTwist& state, const rclcpp::Time&) {
  if (state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  Eigen::Vector3d linear = state.get_linear_velocity();
  Eigen::Vector3d angular = state.get_angular_velocity();

  msg.linear.x = linear(0);
  msg.linear.y = linear(1);
  msg.linear.z = linear(2);

  msg.angular.x = angular(0);
  msg.angular.y = angular(1);
  msg.angular.z = angular(2);
}

void write_msg(geometry_msgs::msg::TwistStamped& msg, const state_representation::DualQuaternionTwist& state, const rclcpp::Time& time) {
  msg.header.stamp = time;
  msg.header.frame_id = state.get_reference_frame();
  write_msg(msg.twist, state, time);
}

void write_msg(tf2_msgs::msg::TFMessage& msg, const state_representation::CartesianState& state, const rclcpp::Time& time) {
  if (state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  geometry_msgs::msg::TransformStamped transform;
  write_msg(transform, state, time);
  msg.transforms.push_back(transform);
}

void write_msg(trajectory_msgs::msg::JointTrajectoryPoint& msg, const state_representation::JointState& state, const rclcpp::Time&) {
  msg.positions = std::vector<double>(state.get_positions().data(), state.get_positions().data() + state.get_positions().size());
  msg.velocities = std::vector<double>(state.get_velocities().data(), state.get_velocities().data() + state.get_velocities().size());
  msg.accelerations = std::vector<double>(state.get_accelerations().data(), state.get_accelerations().data() + state.get_accelerations().size());
  msg.effort = std::vector<double>(state.get_torques().data(), state.get_torques().data() + state.get_torques().size());
}

void write_msg(trajectory_msgs::msg::JointTrajectory& msg, const state_representation::Trajectory<state_representation::JointState>& state, const rclcpp::Time& time) {
  if (state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
  msg.header.frame_id = state.get_name();
  msg.joint_names = state.get_joint_names();

  int trajectory_size = state.get_size();
  msg.points.resize(trajectory_size);

  for (int i = 0; i < trajectory_size; i++) {
    auto point = state[i];
    write_msg(msg.points[i], point.first, time);
    msg.points[i].time_from_start = rclcpp::Duration(point.second);
  }
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

void write_msg(std_msgs::msg::String& msg, const state_representation::State& state, const rclcpp::Time&) {
  msg.data = clproto::encode(state);
}
}// namespace modulo::core::communication::state_conversion
