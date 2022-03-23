#include "modulo_new_core/translators/ReadStateConversion.hpp"

using namespace state_representation::exceptions;

namespace modulo_new_core::translators {

static Eigen::Vector3d read_point(const geometry_msgs::msg::Point& msg) {
  return {msg.x, msg.y, msg.z};
}

static Eigen::Vector3d read_vector3(const geometry_msgs::msg::Vector3& msg) {
  return {msg.x, msg.y, msg.z};
}

static Eigen::Quaterniond read_quaternion(const geometry_msgs::msg::Quaternion& msg) {
  return {msg.w, msg.x, msg.y, msg.z};
}

void read_msg(state_representation::CartesianState& state, const geometry_msgs::msg::Accel& msg) {
  state.set_linear_acceleration(read_vector3(msg.linear));
  state.set_angular_acceleration(read_vector3(msg.angular));
}

void read_msg(state_representation::CartesianState& state, const geometry_msgs::msg::AccelStamped& msg) {
  state.set_reference_frame(msg.header.frame_id);
  read_msg(state, msg.accel);
}

void read_msg(state_representation::CartesianState& state, const geometry_msgs::msg::Pose& msg) {
  state.set_position(read_point(msg.position));
  state.set_orientation(read_quaternion(msg.orientation));
}

void read_msg(state_representation::CartesianState& state, const geometry_msgs::msg::PoseStamped& msg) {
  state.set_reference_frame(msg.header.frame_id);
  read_msg(state, msg.pose);
}

void read_msg(state_representation::CartesianState& state, const geometry_msgs::msg::Transform& msg) {
  state.set_position(read_vector3(msg.translation));
  state.set_orientation(read_quaternion(msg.rotation));
}

void read_msg(state_representation::CartesianState& state, const geometry_msgs::msg::TransformStamped& msg) {
  state.set_reference_frame(msg.header.frame_id);
  state.set_name(msg.child_frame_id);
  read_msg(state, msg.transform);
}

void read_msg(state_representation::CartesianState& state, const geometry_msgs::msg::Twist& msg) {
  state.set_linear_velocity(read_vector3(msg.linear));
  state.set_angular_velocity(read_vector3(msg.angular));
}

void read_msg(state_representation::CartesianState& state, const geometry_msgs::msg::TwistStamped& msg) {
  state.set_reference_frame(msg.header.frame_id);
  read_msg(state, msg.twist);
}

void read_msg(state_representation::CartesianState& state, const geometry_msgs::msg::Wrench& msg) {
  state.set_force(read_vector3(msg.force));
  state.set_torque(read_vector3(msg.torque));
}

void read_msg(state_representation::CartesianState& state, const geometry_msgs::msg::WrenchStamped& msg) {
  state.set_reference_frame(msg.header.frame_id);
  read_msg(state, msg.wrench);
}

void read_msg(state_representation::JointState& state, const sensor_msgs::msg::JointState& msg) {
  state.set_names(msg.name);
  if (!msg.position.empty()) {
    state.set_positions(Eigen::VectorXd::Map(msg.position.data(), msg.position.size()));
  }
  if (!msg.velocity.empty()) {
    state.set_velocities(Eigen::VectorXd::Map(msg.velocity.data(), msg.velocity.size()));
  }
  if (!msg.effort.empty()) {
    state.set_torques(Eigen::VectorXd::Map(msg.effort.data(), msg.effort.size()));
  }
}
}// namespace modulo_new_core::translators