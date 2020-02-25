#include <gtest/gtest.h>
#include <fstream>
#include <unistd.h>
#include "state_representation/Trajectories/Trajectory.hpp"
#include "state_representation/Space/Cartesian/CartesianState.hpp"
#include "state_representation/Robot/JointState.hpp"

TEST(CreateTrajectory, PositiveNos)
{
	//Test if trajectory lists are empty
	EXPECT_TRUE(true);
}

TEST(AddPoint, PositiveNos)
{
	//Idea: compare size of the list before and after add_point
	EXPECT_TRUE(true);
}

TEST(OverloadIndex, PositiveNos)
{
	EXPECT_TRUE(true);
}

int main(int argc, char **argv) 
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}