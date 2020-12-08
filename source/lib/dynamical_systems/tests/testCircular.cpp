#include "dynamical_systems/Circular.hpp"
#include <gtest/gtest.h>

class CircularDSTest : public testing::Test {
protected:
  void SetUp() override {
    current_pose = StateRepresentation::CartesianPose("robot", 10 * Eigen::Vector3d::Random());
    center = StateRepresentation::CartesianPose::Identity("robot");
  }

  StateRepresentation::CartesianPose current_pose;
  StateRepresentation::CartesianPose center;
  double radius = 10;
  unsigned int nb_steps = 100;
  double dt = 0.1;
  double linear_tol = 1e-3;
};

TEST_F(CircularDSTest, TestPositionOnRadius) {
  DynamicalSystems::Circular circularDS(center);
  circularDS.set_radius(radius);

  for (unsigned int i = 0; i < nb_steps; ++i) {
    StateRepresentation::CartesianTwist twist = circularDS.evaluate(current_pose);
    current_pose += dt * twist;
  }

  EXPECT_NEAR((current_pose.get_position() - center.get_position()).norm(), radius, linear_tol);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}