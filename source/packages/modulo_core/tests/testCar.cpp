#include "modulo_core/Action.hpp"
#include "modulo_core/Monitor.hpp"
#include "modulo_core/Visualizer.hpp"
#include "dynamical_systems/Linear.hpp"
#include "rcutils/cmdline_parser.h"
#include <eigen3/Eigen/Core>
#include <iostream>
#include <exception>

class MoveAction : public Modulo::Actions::Action<StateRepresentation::CartesianState>
{
private:
	std::shared_ptr<StateRepresentation::CartesianPose> target_pose;

public:
	explicit MoveAction(const std::string & node_name, const std::chrono::milliseconds & period) :
	Action<StateRepresentation::CartesianState>(std::make_shared<StateRepresentation::CartesianPose>("car_test"), std::make_shared<StateRepresentation::CartesianTwist>("car_test"), node_name, period, true),
	target_pose(std::make_shared<StateRepresentation::CartesianPose>("attractor", Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(0,0,1,0)))
	{}

	void on_configure()
	{
		this->add_subscription<nav_msgs::msg::Odometry>("/demo/odom_demo", this->get_input_state());
		this->add_subscription<geometry_msgs::msg::PoseStamped>("/ds/attractor", this->target_pose, std::chrono::milliseconds(0));
		this->add_publisher<geometry_msgs::msg::Pose>("/demo/cmd_demo", this->get_output_state());

		std::shared_ptr<DynamicalSystems::Linear<StateRepresentation::CartesianState> > move_dynamic = std::make_shared<DynamicalSystems::Linear<StateRepresentation::CartesianState> >(1);
		move_dynamic->set_attractor(this->target_pose);
		this->set_dynamic(move_dynamic);
	}
};

class RandomAttractor : public Modulo::Core::Cell
{
private:
	std::shared_ptr<StateRepresentation::CartesianPose> target_pose;

public:
	explicit RandomAttractor(const std::string & node_name, const std::chrono::milliseconds & period):
	Cell(node_name, period, true),
	target_pose(std::make_shared<StateRepresentation::CartesianPose>("attractor"))
	{}

	void on_configure()
	{
		this->add_publisher<geometry_msgs::msg::PoseStamped>("/ds/attractor", target_pose);
		//this->add_publisher<geometry_msgs::msg::Pose>("/demo/cmd_demo", target_pose);
	}

	void step()
	{
		Eigen::Vector3d init = Eigen::Vector3d::Random()*5;
		init(1) = 0;
		init(2) = 0;
		//this->target_pose->set_position(Eigen::Vector3d::Random() * 5);
		this->target_pose->set_position(init);
		this->send_transform(this->target_pose);
	}
};

class ConsoleVisualizer : public Modulo::Visualizers::Visualizer
{
private:
	//std::shared_ptr<StateRepresentation::CartesianPose> car_pose;
	//std::shared_ptr<StateRepresentation::CartesianTwist> car_twist;
	std::shared_ptr<StateRepresentation::CartesianState> odometry;

public:
	explicit ConsoleVisualizer(const std::string & node_name, const std::chrono::milliseconds & period) :
	Visualizer(node_name, period, true),
	odometry(std::make_shared<StateRepresentation::CartesianState>("/demo/odom_demo"))
	{}

	void on_configure()
	{
		this->add_subscription<nav_msgs::msg::Odometry>("/demo/odom_demo", this->odometry);
	}

	void step()
	{
		std::ostringstream os;
		os << std::endl;
		os << "##### ODOMETRY #####" << std::endl;
		os << *this->odometry << std::endl;

		RCLCPP_INFO(get_logger(), "%s", os.str().c_str());
	}
};

// class SimulatedRobotInterface : public Modulo::Core::Cell
// {
// private:
// 	std::shared_ptr<StateRepresentation::CartesianPose> car_pose;
// 	std::shared_ptr<StateRepresentation::CartesianTwist> desired_twist;
// 	std::shared_ptr<StateRepresentation::CartesianPose> fixed_transform;
// 	std::shared_ptr<StateRepresentation::CartesianState> odometry;

// 	std::chrono::milliseconds dt;

// public:
// 	explicit SimulatedRobotInterface(const std::string & node_name, const std::chrono::milliseconds & period):
// 	Cell(node_name, period, true),
// 	car_pose(std::make_shared<StateRepresentation::CartesianPose>("car_test", Eigen::Vector3d::Random(), Eigen::Quaterniond::UnitRandom())),
// 	desired_twist(std::make_shared<StateRepresentation::CartesianTwist>("car_test")),
// 	dt(period)
// 	{}

// 	void on_configure()
// 	{
// 		this->add_subscription<geometry_msgs::msg::TwistStamped>("/ds/desired_twist", this->desired_twist);
// 		this->add_subscription<nav_msgs::msg::Odometry>("/demo/odom_demo", this->odometry);
// 		this->add_publisher<geometry_msgs::msg::PoseStamped>("/demo/cmd_demo", this->car_pose, std::chrono::milliseconds(0));
// 	}

// 	void step()
// 	{
// 		if(!this->desired_twist->is_empty())
// 		{
// 			*this->car_pose = dt * *this->desired_twist + *this->car_pose;
// 		}
// 		this->send_transform(this->car_pose);
// 	}
// };


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
	const std::chrono::milliseconds period(100);
	const std::chrono::milliseconds period_visualization(100);
	const std::chrono::milliseconds period_monitor(1000);
	const std::chrono::milliseconds period_randomization(10000);

	std::shared_ptr<MoveAction> ma = std::make_shared<MoveAction>("move_action", period);
	std::shared_ptr<RandomAttractor> ra = std::make_shared<RandomAttractor>("random_attractor", period_randomization);
	std::shared_ptr<ConsoleVisualizer> cv = std::make_shared<ConsoleVisualizer>("visualizer", period_visualization);
	//std::shared_ptr<SimulatedRobotInterface> sri = std::make_shared<SimulatedRobotInterface>("robot_interface", period);

	exe.add_node(ma->get_node_base_interface());
	exe.add_node(ra->get_node_base_interface());
	exe.add_node(cv->get_node_base_interface());
	//exe.add_node(sri->get_node_base_interface());

	// std::list<std::string> monitored_nodes = {"move_action", "random_attractor", "robot_interface"}; //Rajouter visualizer ?
	// std::shared_ptr<Modulo::Monitors::Monitor> mo = std::make_shared<Modulo::Monitors::Monitor>("monitor", monitored_nodes, period_monitor);
	// exe.add_node(mo->get_node_base_interface());

	exe.spin();

	rclcpp::shutdown();

	return 0;
}