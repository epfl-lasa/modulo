#include "modulo_core/MotionGenerator.hpp"
#include "modulo_core/Visualizer.hpp"
#include "rcutils/cmdline_parser.h"
#include <eigen3/Eigen/Core>
#include <iostream>

class LinearMotionGenerator : public ModuloCore::MotionGenerator
{
public:
	std::shared_ptr<StateRepresentation::JointState> current_pose;
	std::shared_ptr<StateRepresentation::JointState> desired_velocity;
	std::shared_ptr<StateRepresentation::JointState> target_pose;

	explicit LinearMotionGenerator(const std::string & node_name, const std::chrono::milliseconds & period) :
	current_pose(std::make_shared<StateRepresentation::JointState>("robot")),
	desired_velocity(std::make_shared<StateRepresentation::JointState>("robot")),
	target_pose(std::make_shared<StateRepresentation::JointState>("target", Eigen::Vector3d(4, 5, 6), Eigen::Quaterniond(0,1,0,0))),
	state_timeout(2 * period)
	{
		this->set_in_joint_state_channel("/robot/joint_state");
		this->set_out_joint_state_channel("/ds/desired_joint_state");
		this->set_srv_target_joint_state_channel("/ds/target_joint_state");

		this->set_target_joint_state(StateRepresentation::JointState(3));
		Eigen::ArrayXd target_positions(3);
		target_positions << 1,2,3;
		this->get_target_joint_state().set_positions(target_positions);
	}

	void step()
	{
		/*StateRepresentation::JointState current_state(this->get_in_joint_state());
		StateRepresentation::JointState target_state(this->get_target_joint_state());
		if(current_state.is_compatible(target_state) && !current_state.is_deprecated(state_timeout))
		{
			Eigen::ArrayXd desired_velocities = -Kp * (current_state.get_positions() - target_state.get_positions());
			current_state.set_velocities(desired_velocities);
			this->set_out_joint_state(current_state);
		}*/
	}
};

class ConsoleVisualizer : public ModuloCore::Visualizer
{
public:

	explicit ConsoleVisualizer(const std::string & node_name, const std::chrono::milliseconds & period) :
	Visualizer(node_name, period)
	{
		//this->set_in_joint_state_channel("/robot/joint_state");
	}

	void step()
	{
		/*std::ostringstream os;
		os << this->get_in_joint_state();
		RCUTILS_LOG_INFO_NAMED(get_name(), os.str().c_str());*/
	}
};

class SimulatedRobotInterface : public ModuloCore::Cell
{
public:
	std::chrono::milliseconds state_timeout;
	StateRepresentation::JointState state;
	double dt;

	explicit SimulatedRobotInterface(const std::string & node_name, const std::chrono::milliseconds & period) :
	Cell(node_name, period),
	state_timeout(2 * period),
	state(3),
	dt(0.001)
	{
	/*	this->set_in_joint_state_channel("/ds/desired_joint_state");
		this->set_out_joint_state_channel("/robot/joint_state");
		this->set_in_joint_state(state);
		this->set_out_joint_state(state);*/
	}

	void step()
	{
	/*	if(!this->get_in_joint_state().is_deprecated(state_timeout)) state.get_positions() += dt * this->get_in_joint_state().get_velocities();
		this->set_out_joint_state(state);*/
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
	const std::chrono::milliseconds period(1000);
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