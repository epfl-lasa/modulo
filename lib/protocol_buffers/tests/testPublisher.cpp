#include "dynamical_systems/Circular.hpp"
#include "protocol_buffers/Publisher.hpp"
#include <gtest/gtest.h>
#include <unistd.h>


TEST(PublishCircular, PositiveNos)
{
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	ProtocolBuffers::Publisher<StateRepresentation::CartesianState> publisher("test");

	DynamicalSystems::Circular<StateRepresentation::CartesianState> circularDS(1);
	DynamicalSystems::Circular<StateRepresentation::CartesianState> circularDS2(0.5);

	StateRepresentation::CartesianPose current_pose("ds1", 10 * Eigen::Vector3d::Random());
	StateRepresentation::CartesianPose current_pose2("ds2", 10 * Eigen::Vector3d::Random());

	StateRepresentation::CartesianPose center("center");
	StateRepresentation::CartesianPose center2("center2", 0, 0, 1);
	double radius = 10;

	circularDS.set_center(center);
	circularDS.set_radius(radius);

	circularDS2.set_center(center2);
	circularDS2.set_radius(radius/2);

	double dt = 0.1;
	
	while(true)
	{
		StateRepresentation::CartesianVelocity velocity = circularDS.evaluate(current_pose);
		StateRepresentation::CartesianVelocity velocity2 = circularDS2.evaluate(current_pose2);

		current_pose += dt * velocity;
		current_pose2 += dt * velocity2;
		usleep(100000);

		publisher.send(current_pose);
		publisher.send(current_pose2);
	}
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}