#include "dynamical_systems/Linear.hpp"
#include <gtest/gtest.h>
#include <vector>

using namespace std::literals::chrono_literals;

class LinearDSTest : public testing::Test {
protected:
  void SetUp() override {
    current_pose = StateRepresentation::CartesianPose::Random("robot");
    target_pose = StateRepresentation::CartesianPose::Random("robot");
  }
  void print_current_and_target_pose() {
    std::cout << current_pose << std::endl;
    std::cout << target_pose << std::endl;
    std::cout << abs(current_pose.get_orientation().dot(target_pose.get_orientation())) << std::endl;
  }

  void test_closeness() {
    print_current_and_target_pose();
    EXPECT_NEAR((current_pose.get_position() - target_pose.get_position()).norm(), 0.0f, linear_tol);
    EXPECT_NEAR(current_pose.get_orientation().angularDistance(target_pose.get_orientation()), 0.0f, angular_tol);
  }

  StateRepresentation::CartesianPose current_pose;
  StateRepresentation::CartesianPose target_pose;
  unsigned int nb_steps = 200;
  std::chrono::milliseconds dt = 100ms;
  double linear_tol = 1e-3;
  double angular_tol = 1e-3;
};

TEST_F(LinearDSTest, PositionOnly) {
  current_pose.set_orientation(Eigen::Quaterniond::Identity());
  target_pose.set_orientation(Eigen::Quaterniond::Identity());
  DynamicalSystems::Linear<StateRepresentation::CartesianState> linearDS(target_pose);

  for (unsigned int i = 0; i < nb_steps; ++i) {
    StateRepresentation::CartesianTwist twist = linearDS.evaluate(current_pose);
    current_pose += dt * twist;
  }

  test_closeness();
}

TEST_F(LinearDSTest, OrientationOnly) {
  current_pose.set_position(Eigen::Vector3d::Zero());
  target_pose.set_position(Eigen::Vector3d::Zero());

  DynamicalSystems::Linear<StateRepresentation::CartesianState> linearDS(target_pose);

  for (unsigned int i = 0; i < nb_steps; ++i) {
    StateRepresentation::CartesianTwist twist = linearDS.evaluate(current_pose);
    current_pose = dt * twist + current_pose;
  }
  test_closeness();
}

TEST_F(LinearDSTest, PositionAndOrientation) {
  DynamicalSystems::Linear<StateRepresentation::CartesianState> linearDS(target_pose);

  for (unsigned int i = 0; i < nb_steps; ++i) {
    StateRepresentation::CartesianTwist twist = linearDS.evaluate(current_pose);
    current_pose = dt * twist + current_pose;
  }

  test_closeness();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}