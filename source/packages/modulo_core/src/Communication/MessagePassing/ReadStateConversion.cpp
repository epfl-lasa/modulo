#include "modulo_core/Communication/MessagePassing/ReadStateConversion.hpp"
#include <state_representation/Exceptions/EmptyStateException.hpp>
#include <state_representation/Exceptions/IncompatibleReferenceFramesException.hpp>

using namespace StateRepresentation::Exceptions;

namespace modulo::core::communication::state_conversion {
void read_msg(StateRepresentation::CartesianState& state, const geometry_msgs::msg::Pose& msg) {
  // transform messages
  Eigen::Vector3d position(msg.position.x, msg.position.y, msg.position.z);
  Eigen::Quaterniond orientation(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  // read_msg the state
  state.set_position(position);
  state.set_orientation(orientation);
}

void read_msg(StateRepresentation::CartesianState& state, const geometry_msgs::msg::PoseStamped& msg) {
  state.set_reference_frame(msg.header.frame_id);
  read_msg(state, msg.pose);
}

void read_msg(StateRepresentation::CartesianState& state, const geometry_msgs::msg::Transform& msg) {
  // transform messages
  Eigen::Vector3d position(msg.translation.x, msg.translation.y, msg.translation.z);
  Eigen::Quaterniond orientation(msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z);
  // read_msg the state
  state.set_position(position);
  state.set_orientation(orientation);
}

void read_msg(StateRepresentation::CartesianState& state, const geometry_msgs::msg::TransformStamped& msg) {
  state.set_reference_frame(msg.header.frame_id);
  state.set_name(msg.child_frame_id);
  read_msg(state, msg.transform);
}

void read_msg(StateRepresentation::CartesianState& state, const geometry_msgs::msg::Twist& msg) {
  // transform messages
  Eigen::Vector3d linear_velocity(msg.linear.x, msg.linear.y, msg.linear.z);
  Eigen::Vector3d angular_velocity(msg.angular.x, msg.angular.y, msg.angular.z);
  // read_msg the state
  state.set_linear_velocity(linear_velocity);
  state.set_angular_velocity(angular_velocity);
}

void read_msg(StateRepresentation::CartesianState& state, const geometry_msgs::msg::TwistStamped& msg) {
  state.set_reference_frame(msg.header.frame_id);
  read_msg(state, msg.twist);
}

void read_msg(StateRepresentation::CartesianState& state, const geometry_msgs::msg::Accel& msg) {
  // transform messages
  Eigen::Vector3d linear_acceleration(msg.linear.x, msg.linear.y, msg.linear.z);
  Eigen::Vector3d angular_acceleration(msg.angular.x, msg.angular.y, msg.angular.z);
  // read_msg the state
  state.set_linear_acceleration(linear_acceleration);
  state.set_angular_acceleration(angular_acceleration);
}

void read_msg(StateRepresentation::CartesianState& state, const geometry_msgs::msg::AccelStamped& msg) {
  state.set_reference_frame(msg.header.frame_id);
  read_msg(state, msg.accel);
}

void read_msg(StateRepresentation::CartesianState& state, const geometry_msgs::msg::Wrench& msg) {
  // transform messages
  Eigen::Vector3d force(msg.force.x, msg.force.y, msg.force.z);
  Eigen::Vector3d torque(msg.torque.x, msg.torque.y, msg.torque.z);
  // read_msg the state
  state.set_force(force);
  state.set_torque(torque);
}

void read_msg(StateRepresentation::CartesianState& state, const geometry_msgs::msg::WrenchStamped& msg) {
  state.set_reference_frame(msg.header.frame_id);
  read_msg(state, msg.wrench);
}

void read_msg(StateRepresentation::CartesianState& state, const nav_msgs::msg::Odometry& msg) {
  state.set_reference_frame(msg.header.frame_id);
  // and odometry message contains a pose with uncertainty
  read_msg(state, msg.pose.pose);
  //and a twist with uncertainty
  read_msg(state, msg.twist.twist);
}

void read_msg(StateRepresentation::JointState& state, const sensor_msgs::msg::JointState& msg) {
  state.set_names(msg.name);
  if (!msg.position.empty()) state.set_positions(Eigen::VectorXd::Map(msg.position.data(), msg.position.size()));
  if (!msg.velocity.empty()) state.set_velocities(Eigen::VectorXd::Map(msg.velocity.data(), msg.velocity.size()));
  if (!msg.effort.empty()) state.set_torques(Eigen::VectorXd::Map(msg.effort.data(), msg.effort.size()));
}

void read_msg(StateRepresentation::Jacobian& state, const modulo_msgs::msg::Jacobian& msg) {
  state.set_nb_rows(msg.nb_dimensions);
  state.set_nb_cols(msg.nb_joints);
  state.set_joint_names(msg.joint_names);
  state.set_data(Eigen::MatrixXd::Map(msg.data.data(), msg.nb_dimensions, msg.nb_joints));
}

void read_msg(StateRepresentation::DualQuaternionPose& state, const geometry_msgs::msg::Pose& msg) {
  Eigen::Quaterniond orientation(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  Eigen::Vector3d position(msg.position.x, msg.position.y, msg.position.z);
  state.set_orientation(orientation);
  state.set_position(position);
}

void read_msg(StateRepresentation::DualQuaternionPose& state, const geometry_msgs::msg::PoseStamped& msg) {
  state.set_reference_frame(msg.header.frame_id);
  read_msg(state, msg.pose);
}

void read_msg(StateRepresentation::DualQuaternionTwist& state, const geometry_msgs::msg::Twist& msg) {
  state.set_linear_velocity(Eigen::Vector3d(msg.linear.x, msg.linear.y, msg.linear.z));
  state.set_angular_velocity(Eigen::Vector3d(msg.angular.x, msg.angular.y, msg.angular.z));
}

void read_msg(StateRepresentation::DualQuaternionTwist& state, const geometry_msgs::msg::TwistStamped& msg) {
  state.set_reference_frame(msg.header.frame_id);
  read_msg(state, msg.twist);
}
}// namespace modulo::core::communication::state_conversion
