#include "modulo_core/MotionGenerator.hpp"
#include "modulo_core/Visualizer.hpp"

#include "rcutils/cmdline_parser.h"
#include <eigen3/Eigen/Core>
#include <iostream>


class LinearMotionGenerator : public Modulo::MotionGenerators::MotionGenerator
{
public:
	std::shared_ptr<StateRepresentation::DualQuaternionPose> current_pose;
	std::shared_ptr<StateRepresentation::DualQuaternionTwist> desired_twist;
	std::shared_ptr<StateRepresentation::DualQuaternionPose> target_pose;
	double Kp;

	explicit LinearMotionGenerator(const std::string & node_name, const std::chrono::milliseconds & period) :
	MotionGenerator(node_name, period, true),
	current_pose(std::make_shared<StateRepresentation::DualQuaternionPose>("robot")),
	desired_twist(std::make_shared<StateRepresentation::DualQuaternionTwist>("desired")),
	target_pose(std::make_shared<StateRepresentation::DualQuaternionPose>("target", Eigen::Vector3d(4,5,6), Eigen::Quaterniond(0,0,0,1))),
	Kp(1)
	{
		this->add_subscription<geometry_msgs::msg::PoseStamped>("/robot/pose", this->current_pose);
		this->add_publisher<geometry_msgs::msg::TwistStamped>("/ds/desired_velocity", this->desired_twist);
	}

	void step()
	{
		if(this->target_pose->is_compatible(*this->current_pose))
		{
			this->desired_twist->set_position(this->current_pose->get_position());
			*this->desired_twist = -Kp * log(*(this->current_pose) * this->target_pose->conjugate());
		}
		else
		{
			this->desired_twist->initialize();
		}
	}
};

class ConsoleVisualizer : public Modulo::Visualizers::Visualizer
{
public:
	std::shared_ptr<StateRepresentation::DualQuaternionPose> robot_pose;
	std::shared_ptr<StateRepresentation::DualQuaternionTwist> desired_velocity;

	explicit ConsoleVisualizer(const std::string & node_name, const std::chrono::milliseconds & period) :
	Visualizer(node_name, period, true),
	robot_pose(std::make_shared<StateRepresentation::DualQuaternionPose>("robot")),
	desired_velocity(std::make_shared<StateRepresentation::DualQuaternionTwist>("desired"))
	{
		this->add_subscription<geometry_msgs::msg::PoseStamped>("/robot/pose", this->robot_pose);
		this->add_subscription<geometry_msgs::msg::TwistStamped>("/ds/desired_velocity", this->desired_velocity);
	}

	void step()
	{
		std::ostringstream os;
		os << std::endl;
		os << "##### ROBOT STATE #####" << std::endl;
		os << *this->robot_pose << std::endl;
		os << "##### DESIRED STATE #####" << std::endl;
		os << *this->desired_velocity;
		RCLCPP_INFO(get_logger(), "%s", os.str().c_str());
	}
};

class SimulatedRobotInterface : public Modulo::Core::Cell
{
public:
	std::shared_ptr<StateRepresentation::DualQuaternionPose> robot_pose;
	std::shared_ptr<StateRepresentation::DualQuaternionTwist> desired_twist;
	double dt;

	explicit SimulatedRobotInterface(const std::string & node_name, const std::chrono::milliseconds & period):
	Cell(node_name, period, true),
	robot_pose(std::make_shared<StateRepresentation::DualQuaternionPose>("robot")),
	desired_twist(std::make_shared<StateRepresentation::DualQuaternionTwist>("desired")),
	dt(0.1)
	{
		this->add_subscription<geometry_msgs::msg::TwistStamped>("/ds/desired_velocity", this->desired_twist);
		this->add_publisher<geometry_msgs::msg::PoseStamped>("/robot/pose", this->robot_pose, std::chrono::milliseconds(10000));
	}

	void step()
	{
		if(this->robot_pose->is_compatible(*this->desired_twist))
		{
			this->desired_twist->set_position(this->robot_pose->get_position());
			*this->robot_pose = (exp((dt / 2) * *this->desired_twist) * *this->robot_pose);
		}
		this->robot_pose->set_position(this->robot_pose->get_position() + Eigen::Vector3d::Random()*0.001);
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
	std::shared_ptr<LinearMotionGenerator> lmg = std::make_shared<LinearMotionGenerator>("linear_motion_generator", period);
	std::shared_ptr<ConsoleVisualizer> cv = std::make_shared<ConsoleVisualizer>("console_visualizer", period);
	std::shared_ptr<SimulatedRobotInterface> sri = std::make_shared<SimulatedRobotInterface>("simulated_robot_interface", period);

	exe.add_node(lmg->get_node_base_interface());
	exe.add_node(cv->get_node_base_interface());
	exe.add_node(sri->get_node_base_interface());

	exe.spin();

	rclcpp::shutdown();

	return 0;
}