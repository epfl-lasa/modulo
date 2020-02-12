#include "modulo_core/Action.hpp"
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
	Action<StateRepresentation::CartesianState>(std::make_shared<StateRepresentation::CartesianPose>("robot_test"), std::make_shared<StateRepresentation::CartesianTwist>("robot_test"), node_name, period, true),
	target_pose(std::make_shared<StateRepresentation::CartesianPose>("robot_test", Eigen::Vector3d(-0.419, -0.0468, 0.15059), Eigen::Quaterniond(-0.04616,-0.124,0.991007,-0.018758)))
	{}

	void on_configure()
	{
		this->add_subscription<geometry_msgs::msg::PoseStamped>("/robot_test/pose", this->get_input_state());
		//this->add_subscription<geometry_msgs::msg::PoseStamped>("/ds/attractor", this->target_pose, std::chrono::milliseconds(0));
		this->add_publisher<geometry_msgs::msg::TwistStamped>("/ds/desired_twist", this->get_output_state());

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
	target_pose(std::make_shared<StateRepresentation::CartesianPose>("robot_test"))
	{}

	void on_configure()
	{
		this->add_publisher<geometry_msgs::msg::PoseStamped>("/ds/attractor", target_pose);
	}

	void step()
	{
		this->target_pose->set_position(Eigen::Vector3d::Random() * 5);
	}
};

class ConsoleVisualizer : public Modulo::Visualizers::Visualizer
{
private:
	std::shared_ptr<StateRepresentation::CartesianPose> robot_pose;
	std::shared_ptr<StateRepresentation::CartesianTwist> desired_twist;
	std::shared_ptr<StateRepresentation::Parameter<double> > ds_gain;

public:
	explicit ConsoleVisualizer(const std::string & node_name, const std::chrono::milliseconds & period) :
	Visualizer(node_name, period, true),
	robot_pose(std::make_shared<StateRepresentation::CartesianPose>("robot_test")),
	desired_twist(std::make_shared<StateRepresentation::CartesianTwist>("robot_test"))
	{}

	void on_configure()
	{
		this->add_subscription<geometry_msgs::msg::PoseStamped>("/robot_test/pose", this->robot_pose);
		this->add_subscription<geometry_msgs::msg::TwistStamped>("/ds/desired_twist", this->desired_twist);
	}

	void step()
	{
		std::ostringstream os;
		os << std::endl;
		os << "##### DS TWIST #####" << std::endl;
		os << *this->desired_twist << std::endl;

		os << "##### ROBOT POSE #####" << std::endl;
		os << *this->robot_pose << std::endl;

		RCLCPP_INFO(get_logger(), "%s", os.str().c_str());
	}
};

class SimulatedRobotInterface : public Modulo::Core::Cell
{
private:
	std::shared_ptr<StateRepresentation::CartesianPose> robot_pose;
	std::shared_ptr<StateRepresentation::CartesianTwist> desired_twist;
	std::shared_ptr<StateRepresentation::CartesianPose> fixed_transform;
	std::chrono::milliseconds dt;

public:
	explicit SimulatedRobotInterface(const std::string & node_name, const std::chrono::milliseconds & period):
	Cell(node_name, period, true),
	robot_pose(std::make_shared<StateRepresentation::CartesianPose>("robot_test", Eigen::Vector3d::Random(), Eigen::Quaterniond::UnitRandom())),
	desired_twist(std::make_shared<StateRepresentation::CartesianTwist>("robot_test")),
	dt(period)
	{}

	void on_configure()
	{
		this->add_subscription<geometry_msgs::msg::TwistStamped>("/ds/desired_twist", this->desired_twist);
		this->add_publisher<geometry_msgs::msg::PoseStamped>("/robot_test/pose", this->robot_pose, std::chrono::milliseconds(0));
	}

	void step()
	{
		if(!this->desired_twist->is_empty())
		{
			*this->robot_pose = dt * *this->desired_twist + *this->robot_pose;
		}
		this->send_transform(this->robot_pose);
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
	const std::chrono::milliseconds period(1);
	const std::chrono::milliseconds period_visualization(100);
	const std::chrono::milliseconds period_randomization(10000);

	std::shared_ptr<MoveAction> ma = std::make_shared<MoveAction>("move_action", period);
	std::shared_ptr<RandomAttractor> ra = std::make_shared<RandomAttractor>("random_attractor", period_randomization);
	std::shared_ptr<ConsoleVisualizer> cv = std::make_shared<ConsoleVisualizer>("visualizer", period_visualization);
	std::shared_ptr<SimulatedRobotInterface> sri = std::make_shared<SimulatedRobotInterface>("robot_interface", period);

	exe.add_node(ma->get_node_base_interface());
	exe.add_node(cv->get_node_base_interface());
	exe.add_node(sri->get_node_base_interface());

	exe.spin();

	rclcpp::shutdown();

	return 0;
}