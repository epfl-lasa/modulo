#include "modulo_core/MotionGenerator.hpp"
#include "modulo_core/Visualizer.hpp"
#include "state_representation/Cartesian/CartesianPose.hpp"
#include "state_representation/Cartesian/CartesianVelocity.hpp"
#include "dynamical_systems/Linear.hpp"
#include "rcutils/cmdline_parser.h"
#include <eigen3/Eigen/Core>
#include <iostream>
#include <exception>

class LinearMotionGenerator : public Modulo::MotionGenerators::MotionGenerator
{
private:
	std::shared_ptr<StateRepresentation::CartesianPose> current_pose;
	std::shared_ptr<StateRepresentation::CartesianVelocity> desired_velocity;
	std::shared_ptr<StateRepresentation::CartesianPose> target_pose;
	DynamicalSystems::Linear<StateRepresentation::CartesianState> motion_generator;

public:
	explicit LinearMotionGenerator(const std::string & node_name, const std::chrono::milliseconds & period) :
	MotionGenerator(node_name, period, true),
	current_pose(std::make_shared<StateRepresentation::CartesianPose>("robot")),
	desired_velocity(std::make_shared<StateRepresentation::CartesianVelocity>("robot")),
	target_pose(std::make_shared<StateRepresentation::CartesianPose>("robot", Eigen::Vector3d(4, 5, 6), Eigen::Quaterniond(0,1,0,0))),
	motion_generator(1)
	{
		this->add_subscription<geometry_msgs::msg::PoseStamped>("/robot/pose", this->current_pose);
		this->add_publisher<geometry_msgs::msg::TwistStamped>("/ds/desired_velocity", this->desired_velocity);
		this->motion_generator.set_attractor(*this->target_pose);
	}

	void step()
	{
		if(!this->current_pose->is_empty())
		{
			*this->desired_velocity = this->motion_generator.evaluate(*this->current_pose);
		}
		else
		{
			this->desired_velocity->initialize();
		}
	}
};

class ConsoleVisualizer : public Modulo::Visualizers::Visualizer
{
private:
	std::shared_ptr<StateRepresentation::CartesianPose> robot_pose;
	std::shared_ptr<StateRepresentation::CartesianVelocity> desired_velocity;

public:
	explicit ConsoleVisualizer(const std::string & node_name, const std::chrono::milliseconds & period) :
	Visualizer(node_name, period, true),
	robot_pose(std::make_shared<StateRepresentation::CartesianPose>("robot")),
	desired_velocity(std::make_shared<StateRepresentation::CartesianVelocity>("robot"))
	{
		this->add_subscription<geometry_msgs::msg::PoseStamped>("/robot/pose", this->robot_pose);
		this->add_subscription<geometry_msgs::msg::TwistStamped>("/ds/desired_velocity", this->desired_velocity);
	}

	void step()
	{
		std::ostringstream os;
		os << std::endl;
		os << "##### ROBOT POSE #####" << std::endl;
		os << *this->robot_pose << std::endl;
		os << "##### DESIRED VELOCITY #####" << std::endl;
		os << *this->desired_velocity << std::endl;

		// look up the robot_base transform
		try
		{
			StateRepresentation::CartesianPose robot_base_pose = this->lookup_transform("robot_base");
			os << "##### ROBOT BASE POSE #####" << std::endl;
			os << robot_base_pose;
			RCLCPP_INFO(get_logger(), "%s", os.str().c_str());
		}
		catch (const std::exception& e)
		{
			RCLCPP_ERROR(get_logger(), e.what());
		}
	}
};

class SimulatedRobotInterface : public Modulo::Core::Cell
{
private:
	std::shared_ptr<StateRepresentation::CartesianPose> robot_pose;
	std::shared_ptr<StateRepresentation::CartesianVelocity> desired_velocity;
	std::shared_ptr<StateRepresentation::CartesianPose> fixed_transform;
	std::chrono::milliseconds dt;

public:
	explicit SimulatedRobotInterface(const std::string & node_name, const std::chrono::milliseconds & period):
	Cell(node_name, period, true),
	robot_pose(std::make_shared<StateRepresentation::CartesianPose>("robot", 0, 0, 0)),
	desired_velocity(std::make_shared<StateRepresentation::CartesianVelocity>("robot")),
	fixed_transform(std::make_shared<StateRepresentation::CartesianPose>("robot_base", 4, 5, 3, "robot")),
	dt(period)
	{
		this->add_subscription<geometry_msgs::msg::TwistStamped>("/ds/desired_velocity", this->desired_velocity);
		this->add_publisher<geometry_msgs::msg::PoseStamped>("/robot/pose", this->robot_pose, std::chrono::milliseconds(0));
		this->add_fixed_transform_broadcaster(fixed_transform);
	}

	void step()
	{
		if(!this->desired_velocity->is_empty())
		{
			*this->robot_pose += dt * *this->desired_velocity;
		}
		this->send_transform(*this->robot_pose);
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
	std::shared_ptr<LinearMotionGenerator> lmg = std::make_shared<LinearMotionGenerator>("motion_generator", period);
	std::shared_ptr<ConsoleVisualizer> cv = std::make_shared<ConsoleVisualizer>("visualizer", period);
	std::shared_ptr<SimulatedRobotInterface> sri = std::make_shared<SimulatedRobotInterface>("robot_interface", period);

	exe.add_node(lmg->get_node_base_interface());
	exe.add_node(cv->get_node_base_interface());
	exe.add_node(sri->get_node_base_interface());

	exe.spin();

	rclcpp::shutdown();

	return 0;
}