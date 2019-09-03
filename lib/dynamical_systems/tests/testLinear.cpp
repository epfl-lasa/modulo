#include "dynamical_systems/Linear.hpp"
#include <gtest/gtest.h>
#include <zmq.hpp>
#include <unistd.h>

TEST(CreateDynamicalSystem, PositiveNos)
{
	DynamicalSystems::Linear<StateRepresentation::CartesianState> linearDS(1);
	ASSERT_EQ(linearDS.get_gain().size(), 4);
}

TEST(EvaluateDynamicalSystemPositionOnly, PositiveNos)
{
	DynamicalSystems::Linear<StateRepresentation::CartesianState> linearDS(1);

	StateRepresentation::CartesianPose current_pose("current", 10 * Eigen::Vector3d::Random());
	StateRepresentation::CartesianPose target_pose("target", 10 * Eigen::Vector3d::Random());

	linearDS.set_attractor(target_pose);

	unsigned int nb_steps = 100;
	double dt = 0.1;

	for(unsigned int i=0; i<nb_steps; ++i)
	{
		StateRepresentation::CartesianVelocity velocity = linearDS.evaluate(current_pose);
		current_pose += dt * velocity;
	}

	for(int i=0; i<3; ++i) ASSERT_NEAR(current_pose.get_position()(i), target_pose.get_position()(i), 0.001);
	ASSERT_NEAR(current_pose.get_orientation().w(), target_pose.get_orientation().w(), 0.001);
	for(int i=0; i<3; ++i) ASSERT_NEAR(current_pose.get_orientation().vec()(i), target_pose.get_orientation().vec()(i), 0.001);
}

TEST(EvaluateDynamicalSystemOrientationOnly, PositiveNos)
{
	DynamicalSystems::Linear<StateRepresentation::CartesianState> linearDS(1);

	StateRepresentation::CartesianPose current_pose("current", Eigen::Vector3d(0,0,0));
	Eigen::Array4d orientation = Eigen::Array4d::Random();
	StateRepresentation::CartesianPose target_pose("target", Eigen::Vector3d(0,0,0), Eigen::Quaterniond(orientation(0), orientation(1), orientation(2), orientation(3)));

	linearDS.set_attractor(target_pose);

	unsigned int nb_steps = 100;
	double dt = 0.1;

	for(unsigned int i=0; i<nb_steps; ++i)
	{
		StateRepresentation::CartesianVelocity velocity = linearDS.evaluate(current_pose);
		current_pose += dt * velocity;
	}

	for(int i=0; i<3; ++i) ASSERT_NEAR(current_pose.get_position()(i), target_pose.get_position()(i), 0.01);
	ASSERT_NEAR(current_pose.get_orientation().w(), target_pose.get_orientation().w(), 0.01);
	for(int i=0; i<3; ++i) ASSERT_NEAR(current_pose.get_orientation().vec()(i), target_pose.get_orientation().vec()(i), 0.01);
}

TEST(EvaluateDynamicalSystem, PositiveNos)
{
	DynamicalSystems::Linear<StateRepresentation::CartesianState> linearDS(1);

	StateRepresentation::CartesianPose current_pose("current", 10 * Eigen::Vector3d::Random());
	Eigen::Array4d orientation = Eigen::Array4d::Random();
	StateRepresentation::CartesianPose target_pose("target", 10 * Eigen::Vector3d::Random(), Eigen::Quaterniond(orientation(0), orientation(1), orientation(2), orientation(3)));

	linearDS.set_attractor(target_pose);

	unsigned int nb_steps = 100;
	double dt = 0.1;

	for(unsigned int i=0; i<nb_steps; ++i)
	{
		StateRepresentation::CartesianVelocity velocity = linearDS.evaluate(current_pose);
		current_pose += dt * velocity;
	}

	for(int i=0; i<3; ++i) ASSERT_NEAR(current_pose.get_position()(i), target_pose.get_position()(i), 0.01);
	ASSERT_NEAR(current_pose.get_orientation().w(), target_pose.get_orientation().w(), 0.01);
	for(int i=0; i<3; ++i) ASSERT_NEAR(current_pose.get_orientation().vec()(i), target_pose.get_orientation().vec()(i), 0.01);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}