#include "modulo_core/Action.hpp"
#include "modulo_core/Monitor.hpp"
#include "modulo_core/Visualizer.hpp"
#include "dynamical_systems/Linear.hpp"
#include "state_representation/Trajectories/Trajectory.hpp"
#include "modulo_core/Communication/MessagePassing/PublisherHandler.hpp"
#include "rcutils/cmdline_parser.h"
#include <eigen3/Eigen/Core>
#include <iostream>
#include <exception>


// class Publisher : public Modulo::Core::Cell
// {
// private: 
// 	std::shared_ptr<StateRepresentation::JointState> point;
	
// public:
// 	explicit Publisher(const std::string & node_name, const std::chrono::milliseconds & period) :
// 	Cell(node_name, period, false),
// 	point(std::make_shared<StateRepresentation::JointState>("robot", 4))
// 	{}

// 	void on_configure()
// 	{
// 		this->add_publisher<trajectory_msgs::msg::JointTrajectoryPoint>("/test", point);
// 	}

// 	void step()
// 	{
// 		Eigen::ArrayXd positions(4);
// 		positions << 0.2, 0.2, 0.2, 0.2;
// 		this->point->set_positions(positions);
// 	}
// };

class Publisher : public Modulo::Core::Cell
{
private: 
	StateRepresentation::JointState point;
	std::shared_ptr<StateRepresentation::Trajectory<StateRepresentation::JointState>> traj;
	
public:
	explicit Publisher(const std::string & node_name, const std::chrono::milliseconds & period) :
	Cell(node_name, period, false),
	point("robot", 6),
	traj(std::make_shared<StateRepresentation::Trajectory<StateRepresentation::JointState>>())
	{}

	void on_configure()
	{
		this->add_publisher<trajectory_msgs::msg::JointTrajectory>("/set_joint_trajectory", traj);
		traj->set_reference_frame("world");
		traj->set_joint_names(6);
	}

	void step()
	{	
		this->traj->clear();
		std::chrono::milliseconds step_period(100);
		Eigen::ArrayXd positions(6);
		Eigen::ArrayXd velocities(6);
		positions << 1, 0, 0, 0, 0, 0;
		velocities << 0, 0, 0, 0, 0, 0;
		this->point.set_positions(positions);
		this->point.set_velocities(velocities);
		this->traj->add_point(this->point, step_period);

		positions << 2, 0, 0, 0, 0, 0;
		velocities << 1, 0, 0, 0, 0, 0;
		this->point.set_positions(positions);
		this->point.set_velocities(velocities);
		this->traj->add_point(this->point, step_period);

		positions << 3, 0, 0, 0, 0, 0;
		velocities << 1, 0, 0, 0, 0, 0;
		this->point.set_positions(positions);
		this->point.set_velocities(velocities);
		this->traj->add_point(this->point, step_period);

		positions << 3, 0, 0, 0, 0, 0;
		velocities << 0, 0, 0, 0, 0, 0;
		this->point.set_positions(positions);
		this->point.set_velocities(velocities);
		this->traj->add_point(this->point, step_period);
	}
};


/**
 * A lifecycle node has the same node API
 * as a regular node. This means we can spawn a
 * node, give it a name and add it to the executor.
 */
int main(int argc, char * argv[])
{
	// force flush of the stdout buffer.
	// this ensures a correct sync of all prints
	// even when executed simultaneously within the launch file.
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);

	rclcpp::executors::SingleThreadedExecutor exe;
	const std::chrono::milliseconds period(10);

	std::shared_ptr<Publisher> pub = std::make_shared<Publisher>("publish", period);

	exe.add_node(pub->get_node_base_interface());
	exe.spin();

	rclcpp::shutdown();

	return 0;
}