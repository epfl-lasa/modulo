#include "modulo_core/MotionGenerator.hpp"
#include "modulo_core/Visualizer.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamical_systems/Linear.hpp"
#include <iostream>

class LinearMotionGenerator : public Modulo::MotionGenerators::MotionGenerator
{
private:
	std::shared_ptr<StateRepresentation::CartesianPose> current_pose;
	std::shared_ptr<StateRepresentation::CartesianPose> target_pose;
	std::shared_ptr<StateRepresentation::JointVelocities> desired_velocities;
	std::shared_ptr<StateRepresentation::JacobianMatrix> jacobian;
	DynamicalSystems::Linear<StateRepresentation::CartesianState> motion_generator;

public:
	explicit LinearMotionGenerator(const std::string & node_name, const std::chrono::milliseconds & period) :
	MotionGenerator(node_name, period, true),
	current_pose(std::make_shared<StateRepresentation::CartesianPose>("robot_end_effector", "robot_base")),
	target_pose(std::make_shared<StateRepresentation::CartesianPose>("robot_end_effector", Eigen::Vector3d::Random(), "robot_base")),
	desired_velocities(std::make_shared<StateRepresentation::JointVelocities>("robot", 6)),
	jacobian(std::make_shared<StateRepresentation::JacobianMatrix>("robot", 6)),
	motion_generator(1)
	{
		this->add_subscription<geometry_msgs::msg::PoseStamped>("/robot/eef_pose", this->current_pose);
		this->add_subscription<modulo_msgs::msg::JacobianMatrix>("/robot/jacobian", this->jacobian);
		this->add_publisher<sensor_msgs::msg::JointState>("/ds/joint_velocities", this->desired_velocities);
		this->motion_generator.set_attractor(*this->target_pose);
	}

	void step()
	{
		if(!this->current_pose->is_empty() && !this->jacobian->is_empty())
		{
			StateRepresentation::CartesianTwist desired_twist = this->motion_generator.evaluate(*this->current_pose);
			// transform it into JointVelocities from the jacobian
			*this->desired_velocities = (*this->jacobian) * desired_twist; 
		}
		else
		{
			this->desired_velocities->initialize();
		}
	}
};

class ConsoleVisualizer : public Modulo::Visualizers::Visualizer
{
private:
	std::shared_ptr<StateRepresentation::CartesianPose> robot_pose;
	std::shared_ptr<StateRepresentation::JointVelocities> desired_velocities;
	std::shared_ptr<StateRepresentation::JacobianMatrix> jacobian;

public:
	explicit ConsoleVisualizer(const std::string & node_name, const std::chrono::milliseconds & period) :
	Visualizer(node_name, period, true),
	robot_pose(std::make_shared<StateRepresentation::CartesianPose>("robot", "robot_base")),
	desired_velocities(std::make_shared<StateRepresentation::JointVelocities>("robot", 6)),
	jacobian(std::make_shared<StateRepresentation::JacobianMatrix>("robot", 6))
	{
		this->add_subscription<geometry_msgs::msg::PoseStamped>("/robot/eef_pose", this->robot_pose);
		this->add_subscription<sensor_msgs::msg::JointState>("/ds/joint_velocities", this->desired_velocities);
		this->add_subscription<modulo_msgs::msg::JacobianMatrix>("/robot/jacobian", this->jacobian);
	}

	void step()
	{
		std::ostringstream os;
		os << std::endl;	
		os << "##### ROBOT POSITIONS #####" << std::endl;
		os << *this->robot_pose << std::endl;
		os << "##### DESIRED VELOCITY #####" << std::endl;
		os << *this->desired_velocities << std::endl;
		os << "##### JACOBIAN #####" << std::endl;
		os << *this->jacobian << std::endl;
		RCLCPP_INFO(get_logger(), "%s", os.str().c_str());
	}
};

class SimulatedRobotInterface : public Modulo::Core::Cell
{
private:
	std::shared_ptr<StateRepresentation::CartesianPose> robot_pose;
	std::shared_ptr<StateRepresentation::JointVelocities> desired_velocities;
	std::shared_ptr<StateRepresentation::JacobianMatrix> jacobian;
	std::chrono::milliseconds dt;

public:
	explicit SimulatedRobotInterface(const std::string & node_name, const std::chrono::milliseconds & period) :
	Cell(node_name, period, true),
	robot_pose(std::make_shared<StateRepresentation::CartesianPose>("robot_end_effector", Eigen::Vector3d::Random(), "robot_base")),
	desired_velocities(std::make_shared<StateRepresentation::JointVelocities>("robot", 6)),
	jacobian(std::make_shared<StateRepresentation::JacobianMatrix>("robot", 6)),
	dt(period)
	{
		this->add_subscription<sensor_msgs::msg::JointState>("/ds/joint_velocities", this->desired_velocities);
		this->add_publisher<geometry_msgs::msg::PoseStamped>("/robot/eef_pose", this->robot_pose);
		this->add_publisher<modulo_msgs::msg::JacobianMatrix>("/robot/jacobian", this->jacobian);
	}

	void step()
	{
		// read the jacobian and robot_pose
		this->jacobian->set_data(Eigen::Matrix<double,6,6>::Identity() + 0.001*Eigen::Matrix<double,6,6>::Random());
		this->robot_pose->set_position(this->robot_pose->get_position() + 0.001*Eigen::Vector3d::Random());
		if(!this->desired_velocities->is_empty())
		{
			*this->robot_pose = dt * ((*this->jacobian) * (*this->desired_velocities));
		}
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