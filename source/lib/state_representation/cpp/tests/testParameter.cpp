#include "state_representation/Parameters/Parameter.hpp"
#include "state_representation/Space/Cartesian/CartesianState.hpp"
#include "state_representation/Space/Cartesian/CartesianPose.hpp"
#include <gtest/gtest.h>

TEST(Conversion, PositiveNos)
{
	using namespace StateRepresentation;
	Parameter<CartesianPose> test1("test", CartesianPose::Random("test"));
	Parameter<CartesianState> test2(test1);
	EXPECT_EQ(test2.get_type(), StateType::PARAMETER_CARTESIANPOSE);

	std::shared_ptr<Parameter<CartesianState>> test3 = std::make_shared<Parameter<CartesianState>>(Parameter<CartesianPose>("test", CartesianPose::Random("test")));
	EXPECT_EQ(test3->get_type(), StateType::PARAMETER_CARTESIANPOSE);	
}

int main(int argc, char **argv) 
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}