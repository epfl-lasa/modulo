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
	Action<StateRepresentation::CartesianState>(std::make_shared<StateRepresentation::CartesianPose>("car_test", "odom_demo"), std::make_shared<StateRepresentation::CartesianTwist>("car_test", "odom_demo"), node_name, period, true),
	target_pose(std::make_shared<StateRepresentation::CartesianPose>("attractor", Eigen::Vector3d::Zero(), "odom_demo"))
	{}

	bool on_configure()
	{
		this->add_subscription<nav_msgs::msg::Odometry>("/demo/odom_demo", this->get_input_state());
		this->add_subscription<geometry_msgs::msg::PoseStamped>("/ds/attractor", this->target_pose, std::chrono::milliseconds(0));
		this->add_publisher<geometry_msgs::msg::Twist>("/demo/cmd_demo", this->get_output_state());

		std::shared_ptr<DynamicalSystems::Linear<StateRepresentation::CartesianState> > move_dynamic = std::make_shared<DynamicalSystems::Linear<StateRepresentation::CartesianState> >(1);
		move_dynamic->set_attractor(this->target_pose);
		this->set_dynamic(move_dynamic);
		return true;
	}
};

class RandomAttractor : public Modulo::Core::Cell
{
private:
	std::shared_ptr<StateRepresentation::CartesianPose> target_pose;

public:
	explicit RandomAttractor(const std::string & node_name, const std::chrono::milliseconds & period):
	Cell(node_name, period, true),
	target_pose(std::make_shared<StateRepresentation::CartesianPose>("attractor", "odom_demo"))
	{}

	bool on_configure()
	{
		this->add_publisher<geometry_msgs::msg::PoseStamped>("/ds/attractor", target_pose, std::chrono::milliseconds(100));
		this->add_asynchronous_transform_broadcaster(target_pose, std::chrono::milliseconds(100));
		return true;
	}

	void step()
	{
		Eigen::Vector3d init = Eigen::Vector3d::Random()*5;
		//init(1) = 0;
		init(2) = 0;
		//this->target_pose->set_position(Eigen::Vector3d::Random() * 5);
		this->target_pose->set_position(init);
	}
};

class ConsoleVisualizer : public Modulo::Visualizers::Visualizer
{
private:
	std::shared_ptr<StateRepresentation::CartesianState> odometry;

public:
	explicit ConsoleVisualizer(const std::string & node_name, const std::chrono::milliseconds & period) :
	Visualizer(node_name, period, true),
	odometry(std::make_shared<StateRepresentation::CartesianState>("odom_demo", "odom_demo"))
	{}

	bool on_configure()
	{
		this->add_subscription<nav_msgs::msg::Odometry>("/demo/odom_demo", this->odometry);
		return true;
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
	const std::chrono::milliseconds period_visualization(1000);
	const std::chrono::milliseconds period_monitor(1000);
	const std::chrono::milliseconds period_randomization(10000);

	std::shared_ptr<MoveAction> ma = std::make_shared<MoveAction>("move_action", period);
	std::shared_ptr<RandomAttractor> ra = std::make_shared<RandomAttractor>("random_attractor", period_randomization);
	std::shared_ptr<ConsoleVisualizer> cv = std::make_shared<ConsoleVisualizer>("visualizer", period_visualization);

	exe.add_node(ma->get_node_base_interface());
	exe.add_node(ra->get_node_base_interface());
	exe.add_node(cv->get_node_base_interface());

	exe.spin();

	rclcpp::shutdown();

	return 0;
}