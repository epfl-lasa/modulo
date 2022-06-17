#include <gtest/gtest.h>

#include "modulo_new_core/translators/message_readers.hpp"
#include "modulo_new_core/translators/message_writers.hpp"

#include <rclcpp/clock.hpp>
#include <state_representation/exceptions/EmptyStateException.hpp>

using namespace modulo_new_core::translators;

template<typename MsgT, typename DataT>
static void test_std_messages(const DataT& state, const rclcpp::Time& time) {
  auto message = MsgT();
  write_message(message, state, time);
  EXPECT_EQ(message.data, state);
  DataT new_state;
  read_message(new_state, message);
  EXPECT_EQ(state, new_state);
}

static void expect_vector_equal(const Eigen::Vector3d& v1, const geometry_msgs::msg::Vector3& v2) {
  EXPECT_FLOAT_EQ(v1.x(), v2.x);
  EXPECT_FLOAT_EQ(v1.y(), v2.y);
  EXPECT_FLOAT_EQ(v1.z(), v2.z);
}

static void expect_point_equal(const Eigen::Vector3d& vector, const geometry_msgs::msg::Point& point) {
  EXPECT_FLOAT_EQ(vector.x(), point.x);
  EXPECT_FLOAT_EQ(vector.y(), point.y);
  EXPECT_FLOAT_EQ(vector.z(), point.z);
}

static void expect_quaternion_equal(const Eigen::Quaterniond& q1, const geometry_msgs::msg::Quaternion& q2) {
  EXPECT_FLOAT_EQ(q1.x(), q2.x);
  EXPECT_FLOAT_EQ(q1.y(), q2.y);
  EXPECT_FLOAT_EQ(q1.z(), q2.z);
  EXPECT_FLOAT_EQ(q1.w(), q2.w);
}

class MessageTranslatorsTest : public ::testing::Test {
protected:
  void SetUp() override {
    state_ = state_representation::CartesianState::Random("test", "reference");
    joint_state_ = state_representation::JointState::Random("robot", 3);
  }
  state_representation::CartesianState state_;
  state_representation::JointState joint_state_;
  rclcpp::Clock clock_;
};

TEST_F(MessageTranslatorsTest, TestAccel) {
  auto accel = geometry_msgs::msg::Accel();
  write_message(accel, state_, clock_.now());
  expect_vector_equal(state_.get_linear_acceleration(), accel.linear);
  expect_vector_equal(state_.get_angular_acceleration(), accel.angular);

  state_representation::CartesianState new_state("new");
  EXPECT_THROW(write_message(accel, new_state, clock_.now()), state_representation::exceptions::EmptyStateException);
  read_message(new_state, accel);
  EXPECT_TRUE(new_state.get_acceleration().isApprox(state_.get_acceleration()));

  auto accel_stamped = geometry_msgs::msg::AccelStamped();
  write_message(accel_stamped, state_, clock_.now());
  EXPECT_EQ(state_.get_reference_frame(), accel_stamped.header.frame_id);
  expect_vector_equal(state_.get_linear_acceleration(), accel_stamped.accel.linear);
  expect_vector_equal(state_.get_angular_acceleration(), accel_stamped.accel.angular);
  new_state = state_representation::CartesianState("new");
  read_message(new_state, accel_stamped);
  EXPECT_EQ(new_state.get_reference_frame(), state_.get_reference_frame());
  EXPECT_TRUE(new_state.get_acceleration().isApprox(state_.get_acceleration()));
}

TEST_F(MessageTranslatorsTest, TestPose) {
  auto pose = geometry_msgs::msg::Pose();
  write_message(pose, state_, clock_.now());
  expect_point_equal(state_.get_position(), pose.position);
  expect_quaternion_equal(state_.get_orientation(), pose.orientation);

  state_representation::CartesianState new_state("new");
  EXPECT_THROW(write_message(pose, new_state, clock_.now()), state_representation::exceptions::EmptyStateException);
  read_message(new_state, pose);
  EXPECT_TRUE(new_state.get_pose().isApprox(state_.get_pose()));

  auto pose_stamped = geometry_msgs::msg::PoseStamped();
  write_message(pose_stamped, state_, clock_.now());
  EXPECT_EQ(state_.get_reference_frame(), pose_stamped.header.frame_id);
  expect_point_equal(state_.get_position(), pose_stamped.pose.position);
  expect_quaternion_equal(state_.get_orientation(), pose_stamped.pose.orientation);
  new_state = state_representation::CartesianState("new");
  read_message(new_state, pose_stamped);
  EXPECT_EQ(new_state.get_reference_frame(), state_.get_reference_frame());
  EXPECT_TRUE(new_state.get_pose().isApprox(state_.get_pose()));
}

TEST_F(MessageTranslatorsTest, TestTransform) {
  auto tf = geometry_msgs::msg::Transform();
  write_message(tf, state_, clock_.now());
  expect_vector_equal(state_.get_position(), tf.translation);
  expect_quaternion_equal(state_.get_orientation(), tf.rotation);

  state_representation::CartesianState new_state("new");
  EXPECT_THROW(write_message(tf, new_state, clock_.now()), state_representation::exceptions::EmptyStateException);
  read_message(new_state, tf);
  EXPECT_TRUE(new_state.get_pose().isApprox(state_.get_pose()));

  auto tf_stamped = geometry_msgs::msg::TransformStamped();
  write_message(tf_stamped, state_, clock_.now());
  EXPECT_EQ(state_.get_name(), tf_stamped.child_frame_id);
  EXPECT_EQ(state_.get_reference_frame(), tf_stamped.header.frame_id);
  expect_vector_equal(state_.get_position(), tf_stamped.transform.translation);
  expect_quaternion_equal(state_.get_orientation(), tf_stamped.transform.rotation);
  new_state = state_representation::CartesianState("new");
  read_message(new_state, tf_stamped);
  EXPECT_EQ(new_state.get_name(), state_.get_name());
  EXPECT_EQ(new_state.get_reference_frame(), state_.get_reference_frame());
  EXPECT_TRUE(new_state.get_pose().isApprox(state_.get_pose()));
}

TEST_F(MessageTranslatorsTest, TestTwist) {
  auto twist = geometry_msgs::msg::Twist();
  write_message(twist, state_, clock_.now());
  expect_vector_equal(state_.get_linear_velocity(), twist.linear);
  expect_vector_equal(state_.get_angular_velocity(), twist.angular);

  state_representation::CartesianState new_state("new");
  EXPECT_THROW(write_message(twist, new_state, clock_.now()), state_representation::exceptions::EmptyStateException);
  read_message(new_state, twist);
  EXPECT_TRUE(new_state.get_twist().isApprox(state_.get_twist()));

  auto twist_stamped = geometry_msgs::msg::TwistStamped();
  write_message(twist_stamped, state_, clock_.now());
  EXPECT_EQ(state_.get_reference_frame(), twist_stamped.header.frame_id);
  expect_vector_equal(state_.get_linear_velocity(), twist_stamped.twist.linear);
  expect_vector_equal(state_.get_angular_velocity(), twist_stamped.twist.angular);
  new_state = state_representation::CartesianState("new");
  read_message(new_state, twist_stamped);
  EXPECT_EQ(new_state.get_reference_frame(), state_.get_reference_frame());
  EXPECT_TRUE(new_state.get_twist().isApprox(state_.get_twist()));
}

TEST_F(MessageTranslatorsTest, TestWrench) {
  auto wrench = geometry_msgs::msg::Wrench();
  write_message(wrench, state_, clock_.now());
  expect_vector_equal(state_.get_force(), wrench.force);
  expect_vector_equal(state_.get_torque(), wrench.torque);

  state_representation::CartesianState new_state("new");
  EXPECT_THROW(write_message(wrench, new_state, clock_.now()), state_representation::exceptions::EmptyStateException);
  read_message(new_state, wrench);
  EXPECT_TRUE(new_state.get_wrench().isApprox(state_.get_wrench()));

  auto wrench_stamped = geometry_msgs::msg::WrenchStamped();
  write_message(wrench_stamped, state_, clock_.now());
  EXPECT_EQ(state_.get_reference_frame(), wrench_stamped.header.frame_id);
  expect_vector_equal(state_.get_force(), wrench_stamped.wrench.force);
  expect_vector_equal(state_.get_torque(), wrench_stamped.wrench.torque);
  new_state = state_representation::CartesianState("new");
  read_message(new_state, wrench_stamped);
  EXPECT_EQ(new_state.get_reference_frame(), state_.get_reference_frame());
  EXPECT_TRUE(new_state.get_wrench().isApprox(state_.get_wrench()));
}

TEST_F(MessageTranslatorsTest, TestJointState) {
  auto message = sensor_msgs::msg::JointState();
  write_message(message, joint_state_, clock_.now());
  for (unsigned int i = 0; i < joint_state_.get_size(); ++i) {
    EXPECT_EQ(message.name.at(i), joint_state_.get_names().at(i));
    EXPECT_NEAR(message.position.at(i), joint_state_.get_position(i), 1e-5);
    EXPECT_NEAR(message.velocity.at(i), joint_state_.get_velocity(i), 1e-5);
    EXPECT_NEAR(message.effort.at(i), joint_state_.get_torque(i), 1e-5);
  }

  auto new_state = state_representation::JointState("test", {"1", "2", "3"});
  EXPECT_THROW(write_message(message, new_state, clock_.now()), state_representation::exceptions::EmptyStateException);
  read_message(new_state, message);
  EXPECT_TRUE(new_state.get_positions().isApprox(joint_state_.get_positions()));
  EXPECT_TRUE(new_state.get_velocities().isApprox(joint_state_.get_velocities()));
  EXPECT_TRUE(new_state.get_torques().isApprox(joint_state_.get_torques()));
  for (unsigned int i = 0; i < joint_state_.get_size(); ++i) {
    EXPECT_EQ(new_state.get_names().at(i), joint_state_.get_names().at(i));
  }
}

TEST_F(MessageTranslatorsTest, TestStdMsgs) {
  test_std_messages<std_msgs::msg::Bool, bool>(true, clock_.now());
  test_std_messages<std_msgs::msg::Float64, double>(0.3, clock_.now());
  test_std_messages<std_msgs::msg::Int32, int>(2, clock_.now());
  test_std_messages<std_msgs::msg::String, std::string>("test", clock_.now());

  auto message = std_msgs::msg::Float64MultiArray();
  std::vector<double> state = {0.3, 0.6};
  write_message(message, state, clock_.now());
  std::vector<double> new_state;
  read_message(new_state, message);
  for (std::size_t i = 0; i < state.size(); ++i) {
    EXPECT_EQ(message.data.at(i), state.at(i));
    EXPECT_EQ(state.at(i), new_state.at(i));
  }
}

TEST_F(MessageTranslatorsTest, TestEncodedState) {
  auto message = modulo_new_core::EncodedState();
  write_message(message, state_, clock_.now());
  state_representation::CartesianState new_state;
  read_message(new_state, message);
  EXPECT_TRUE(state_.data().isApprox(new_state.data()));
  EXPECT_EQ(state_.get_name(), new_state.get_name());
  EXPECT_EQ(state_.get_reference_frame(), new_state.get_reference_frame());
}

TEST_F(MessageTranslatorsTest, TestEncodedStatePointer) {
  auto state_ptr = state_representation::make_shared_state(state_);
  auto message = modulo_new_core::EncodedState();
  write_message(message, state_ptr, clock_.now());
  auto new_state_ptr = state_representation::make_shared_state(state_representation::CartesianState());
  read_message(new_state_ptr, message);
  auto new_state = *std::dynamic_pointer_cast<state_representation::CartesianState>(new_state_ptr);
  EXPECT_TRUE(state_.data().isApprox(new_state.data()));
  EXPECT_EQ(state_.get_name(), new_state.get_name());
  EXPECT_EQ(state_.get_reference_frame(), new_state.get_reference_frame());
}