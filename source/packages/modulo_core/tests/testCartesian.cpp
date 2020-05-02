#include "modulo_core/MotionGenerator.hpp"
#include "modulo_core/Visualizer.hpp"
#include "dynamical_systems/Linear.hpp"
#include "rcutils/cmdline_parser.h"
#include <eigen3/Eigen/Core>
#include <iostream>
#include <exception>

class LinearMotionGenerator : public Modulo::MotionGenerators::MotionGenerator
{
private:
	std::shared_ptr<StateRepresentation::CartesianPose> current_pose;
	std::shared_ptr<StateRepresentation::CartesianTwist> desired_twist;
	std::shared_ptr<StateRepresentation::CartesianPose> target_pose;
	std::shared_ptr<StateRepresentation::Parameter<double> > gain;
	DynamicalSystems::Linear<StateRepresentation::CartesianState> motion_generator;

public:
	explicit LinearMotionGenerator(const std::string & node_name, const std::chrono::milliseconds & period) :
	MotionGenerator(node_name, period, true),
	current_pose(std::make_shared<StateRepresentation::CartesianPose>("robot_test")),
	desired_twist(std::make_shared<StateRepresentation::CartesianTwist>("robot_test")),
	target_pose(std::make_shared<StateRepresentation::CartesianPose>("robot_test", Eigen::Vector3d(-0.419, -0.0468, 0.15059), Eigen::Quaterniond(-0.04616,-0.124,0.991007,-0.018758))),
	gain(std::make_shared<StateRepresentation::Parameter<double> >("ds_gain", 42)),
	motion_generator(1)
	{}

	bool on_configure()
	{
		this->add_subscription<geometry_msgs::msg::PoseStamped>("/robot_test/pose", this->current_pose);
		this->add_publisher<geometry_msgs::msg::TwistStamped>("/ds/desired_twist", this->desired_twist);
		this->add_publisher<std_msgs::msg::Float64>("/ds/gain", this->gain, 0);
		this->motion_generator.set_attractor(*this->target_pose);
		return true;
	}

	void step()
	{
		if(!this->current_pose->is_empty())
		{
			*this->desired_twist = this->motion_generator.evaluate(*this->current_pose);
		}
		else
		{
			this->desired_twist->initialize();
		}
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
	desired_twist(std::make_shared<StateRepresentation::CartesianTwist>("robot_test")),
	ds_gain(std::make_shared<StateRepresentation::Parameter<double> >("ds_gain"))
	{}

	bool on_configure()
	{
		this->add_subscription<geometry_msgs::msg::PoseStamped>("/robot_test/pose", this->robot_pose);
		this->add_subscription<geometry_msgs::msg::TwistStamped>("/ds/desired_twist", this->desired_twist);
		this->add_subscription<std_msgs::msg::Float64>("/ds/gain", this->ds_gain);
		return true;
	}

	void step()
	{
		std::ostringstream os;
		os << std::endl;
		os << "##### ROBOT POSE #####" << std::endl;
		os << *this->robot_pose << std::endl;

		os << "##### DS GAIN #####" << std::endl;
		os << this->ds_gain->get_value() << std::endl;
		//os << "##### DESIRED VELOCITY #####" << std::endl;
		//os << *this->desired_twist << std::endl;

		// look up the robot_base transform
		/*try
		{
			StateRepresentation::CartesianPose robot_base_pose = this->lookup_transform("robot_base");
			os << "##### ROBOT BASE POSE #####" << std::endl;
			os << robot_base_pose;
		}
		catch (const std::exception& e)
		{
			RCLCPP_ERROR(get_logger(), e.what());
		}*/

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
	fixed_transform(std::make_shared<StateRepresentation::CartesianPose>("robot_base", 4, 5, 3, "robot_test")),
	dt(period)
	{}

	bool on_configure()
	{
		this->add_subscription<geometry_msgs::msg::TwistStamped>("/ds/desired_twist", this->desired_twist);
		this->add_publisher<geometry_msgs::msg::PoseStamped>("/robot_test/pose", this->robot_pose, 0);
		//this->add_fixed_transform_broadcaster(fixed_transform);
		return true;
	}

	void step()
	{
		if(!this->desired_twist->is_empty())
		{
			*this->robot_pose = dt * *this->desired_twist + *this->robot_pose;
		}
		this->send_transform(*this->robot_pose);
		this->send_transform(*this->fixed_transform);
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

	std::shared_ptr<LinearMotionGenerator> lmg = std::make_shared<LinearMotionGenerator>("motion_generator", period);
	std::shared_ptr<ConsoleVisualizer> cv = std::make_shared<ConsoleVisualizer>("visualizer", period_visualization);
	std::shared_ptr<SimulatedRobotInterface> sri = std::make_shared<SimulatedRobotInterface>("robot_interface", period);

	exe.add_node(lmg->get_node_base_interface());
	exe.add_node(cv->get_node_base_interface());
	exe.add_node(sri->get_node_base_interface());

	exe.spin();

	rclcpp::shutdown();

	return 0;
}