#include "state_representation/Cartesian/CartesianPose.hpp"
#include "state_representation/Cartesian/CartesianVelocity.hpp"
#include <gtest/gtest.h>
#include <fstream>
#include <zmq.hpp>
#include <unistd.h>


TEST(MultiplyTransformsBothOperators, PositiveNos)
{
	Eigen::Vector3d pos1(1,2,3);
	Eigen::Quaterniond rot1(1,0,0,0);
	StateRepresentation::CartesianPose tf1("t1", pos1, rot1);

	Eigen::Vector3d pos2(4,5,6);
	Eigen::Quaterniond rot2(1,0,0,0);
	StateRepresentation::CartesianPose tf2("t2", pos2, rot2, "t1");

	StateRepresentation::CartesianPose tf3 = tf1 * tf2;
	tf1 *= tf2;
	
	EXPECT_EQ(tf3.get_name(), "t2");
	for(int i=0; i<tf1.get_position().size(); ++i) EXPECT_NEAR(tf1.get_position()(i), tf3.get_position()(i), 0.00001);
}


TEST(MultiplyTransformsSameOrientation, PositiveNos)
{
	Eigen::Vector3d pos1(1,2,3);
	Eigen::Quaterniond rot1(1,0,0,0);
	StateRepresentation::CartesianPose tf1("t1", pos1, rot1);

	Eigen::Vector3d pos2(4,5,6);
	Eigen::Quaterniond rot2(1,0,0,0);
	StateRepresentation::CartesianPose tf2("t2", pos2, rot2, "t1");

	tf1 *= tf2;
	
	Eigen::Vector3d pos_truth(5,7,9);
	for(int i=0; i<pos_truth.size(); ++i) EXPECT_NEAR(tf1.get_position()(i), pos_truth(i), 0.00001);
}

TEST(MultiplyTransformsDifferentOrientation, PositiveNos)
{
	Eigen::Vector3d pos1(1,2,3);
	Eigen::Quaterniond rot1(0.70710678, 0.70710678, 0., 0.);
	StateRepresentation::CartesianPose tf1("t1", pos1, rot1);

	Eigen::Vector3d pos2(4,5,6);
	Eigen::Quaterniond rot2(0., 0., 0.70710678, 0.70710678);
	StateRepresentation::CartesianPose tf2("t2", pos2, rot2, "t1");

	tf1 *= tf2;
	
	Eigen::Vector3d pos_truth(5,-4,8);
	Eigen::Quaterniond rot_truth(0.,0.,0.,1.);

	std::cerr << "position" << std::endl;
	std::cerr << tf1.get_position() << std::endl;
	std::cerr << "orientation" << std::endl;
	std::cerr << tf1.get_orientation().coeffs() << std::endl;

	for(int i=0; i<pos_truth.size(); ++i) EXPECT_NEAR(tf1.get_position()(i), pos_truth(i), 0.00001);
    for(int i=0; i<4; ++i) EXPECT_NEAR(tf1.get_orientation().coeffs()(i), rot_truth.coeffs()(i), 0.00001);
}

TEST(TestInverseNullOrientation, PositiveNos)
{
	Eigen::Vector3d pos1(1,2,3);
	Eigen::Quaterniond rot1(1., 0., 0., 0.);
	StateRepresentation::CartesianPose tf1("t1", pos1, rot1);

	tf1 = tf1.inverse();
	
	Eigen::Vector3d pos_truth(-1,-2,-3);
	Eigen::Quaterniond rot_truth(1.,0.,0.,0.);

	std::cerr << "position" << std::endl;
	std::cerr << tf1.get_position() << std::endl;
	std::cerr << "orientation" << std::endl;
	std::cerr << tf1.get_orientation().coeffs() << std::endl;

	EXPECT_EQ(tf1.get_name(), "world");
	EXPECT_EQ(tf1.get_reference_frame(), "t1");
	for(int i=0; i<pos_truth.size(); ++i) EXPECT_NEAR(tf1.get_position()(i), pos_truth(i), 0.00001);
    for(int i=0; i<4; ++i) EXPECT_NEAR(tf1.get_orientation().coeffs()(i), rot_truth.coeffs()(i), 0.00001);
}

TEST(TestInverseNonNullOrientation, PositiveNos)
{
	Eigen::Vector3d pos1(1,2,3);
	Eigen::Quaterniond rot1(0.70710678, 0.70710678, 0., 0.);
	StateRepresentation::CartesianPose tf1("t1", pos1, rot1);

	tf1 = tf1.inverse();
	
	Eigen::Vector3d pos_truth(-1,-3,2);
	Eigen::Quaterniond rot_truth(0.70710678, -0.70710678, 0., 0.);

	std::cerr << "position" << std::endl;
	std::cerr << tf1.get_position() << std::endl;
	std::cerr << "orientation" << std::endl;
	std::cerr << tf1.get_orientation().coeffs() << std::endl;

	for(int i=0; i<pos_truth.size(); ++i) EXPECT_NEAR(tf1.get_position()(i), pos_truth(i), 0.00001);
    for(int i=0; i<4; ++i) EXPECT_NEAR(tf1.get_orientation().coeffs()(i), rot_truth.coeffs()(i), 0.00001);
}

TEST(TestMultiplyInverseNonNullOrientation, PositiveNos)
{
	Eigen::Vector3d pos1(1,2,3);
	Eigen::Quaterniond rot1(0.70710678, 0.70710678, 0., 0.);
	StateRepresentation::CartesianPose tf1("t1", pos1, rot1);

	tf1 *= tf1.inverse();
	
	Eigen::Vector3d pos_truth(0,0,0);
	Eigen::Quaterniond rot_truth(1., 0., 0., 0.);

	std::cerr << "position" << std::endl;
	std::cerr << tf1.get_position() << std::endl;
	std::cerr << "orientation" << std::endl;
	std::cerr << tf1.get_orientation().coeffs() << std::endl;

	for(int i=0; i<pos_truth.size(); ++i) EXPECT_NEAR(tf1.get_position()(i), pos_truth(i), 0.00001);
    for(int i=0; i<4; ++i) EXPECT_NEAR(tf1.get_orientation().coeffs()(i), rot_truth.coeffs()(i), 0.00001);
}

TEST(TestAddTwoPoses, PositiveNos)
{
	Eigen::Vector3d pos1 = Eigen::Vector3d::Zero();
	Eigen::Quaterniond rot1 = Eigen::Quaterniond::Identity(); 
	StateRepresentation::CartesianPose tf1("t1", pos1, rot1);

	Eigen::Vector3d pos2(1,0,0);
	Eigen::Quaterniond rot2(0,1,0,0); 
	StateRepresentation::CartesianPose tf2("t1", pos2, rot2);

	std::cout << tf1 + tf2 << std::endl;
	std::cout << tf1 - tf2 << std::endl;
}

TEST(TestAddDisplacement, PositiveNos)
{
	Eigen::Vector3d pos1 = Eigen::Vector3d::Zero();
	Eigen::Quaterniond rot1 = Eigen::Quaterniond::Identity(); 
	StateRepresentation::CartesianPose tf1("t1", pos1, rot1);

	StateRepresentation::CartesianVelocity vel("t1");
	vel.set_linear_velocity(Eigen::Vector3d(0.1,0.1,0.1));
	vel.set_angular_velocity(Eigen::Vector3d(0.1,0.1,0));

	std::chrono::milliseconds dt1(10);
	std::cout << tf1 + dt1 * vel << std::endl;

	std::chrono::milliseconds dt2(1000);
	std::cout << tf1 + dt2 * vel << std::endl;

	std::chrono::seconds dt3(1);
	std::cout << tf1 + dt3 * vel << std::endl;
}

TEST(TestPoseToVelocity, PositiveNos)
{
	Eigen::Vector3d pos1 = Eigen::Vector3d::Zero();
	Eigen::Quaterniond rot1 = Eigen::Quaterniond::Identity(); 
	StateRepresentation::CartesianPose tf1("t1", pos1, rot1);

	Eigen::Vector3d pos2(1,0,0);
	Eigen::Quaterniond rot2(0,1,0,0); 
	StateRepresentation::CartesianPose tf2("t1", pos2, rot2);

	std::chrono::seconds dt1(1);
	std::cout << (tf1 - tf2) / dt1 << std::endl;

	std::chrono::seconds dt2(10);
	std::cout << (tf1 - tf2) / dt2 << std::endl;

	std::chrono::milliseconds dt3(100);
	std::cout << (tf1 - tf2) / dt3 << std::endl;
}

TEST(TestImplicitConversion, PositiveNos)
{
	Eigen::Vector3d pos1 = Eigen::Vector3d::Zero();
	Eigen::Quaterniond rot1 = Eigen::Quaterniond::Identity(); 
	StateRepresentation::CartesianPose tf1("t1", pos1, rot1);

	StateRepresentation::CartesianVelocity vel("t1");
	vel.set_linear_velocity(Eigen::Vector3d(0.1,0.1,0.1));
	vel.set_angular_velocity(Eigen::Vector3d(0.1,0.1,0));

	tf1 += vel;

	std::cout << tf1 << std::endl;

}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}