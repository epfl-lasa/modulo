#include "modulo_core/Cell.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamical_systems/Linear.hpp"
#include <iostream>

namespace
{
	class LinearMotionGenerator : public Modulo::Core::Cell
	{
	private:
		std::shared_ptr<StateRepresentation::JointState> current_positions;
		std::shared_ptr<StateRepresentation::JointState> desired_velocities;
		DynamicalSystems::Linear<StateRepresentation::JointState> motion_generator;

	public:
		explicit LinearMotionGenerator(const std::string & node_name, const std::chrono::milliseconds & period) :
		Cell(node_name, period),
		current_positions(std::make_shared<StateRepresentation::JointState>("robot", 6)),
		desired_velocities(std::make_shared<StateRepresentation::JointState>("robot", 6)),
		motion_generator(StateRepresentation::JointPositions("robot", Eigen::VectorXd::Random(6)))
		{
			this->add_parameters(this->motion_generator.get_parameters());
		}

		bool on_configure()
		{
			this->add_subscription<sensor_msgs::msg::JointState>("/robot/joint_state", this->current_positions);
			this->add_publisher<sensor_msgs::msg::JointState>("/ds/desired_velocities", this->desired_velocities);
			return true;
		}

		void step()
		{
			if(!this->current_positions->is_empty())
			{
				*this->desired_velocities = this->motion_generator.evaluate(*this->current_positions);
			}
			else
			{
				this->desired_velocities->initialize();
			}
		}
	};

	class ConsoleVisualizer : public Modulo::Core::Cell
	{
	private:
		std::shared_ptr<StateRepresentation::JointState> robot_positions;
		std::shared_ptr<StateRepresentation::JointState> desired_velocities;

	public:
		explicit ConsoleVisualizer(const std::string & node_name, const std::chrono::milliseconds & period) :
		Cell(node_name, period),
		robot_positions(std::make_shared<StateRepresentation::JointState>("robot", 6)),
		desired_velocities(std::make_shared<StateRepresentation::JointState>("robot", 6))
		{}

		bool on_configure()
		{
			this->add_subscription<sensor_msgs::msg::JointState>("/robot/joint_state", this->robot_positions);
			this->add_subscription<sensor_msgs::msg::JointState>("/ds/desired_velocities", this->desired_velocities);
			return true;
		}

		void step()
		{
			std::ostringstream os;		
			os << "##### ROBOT POSITIONS #####" << std::endl;
			os << *this->robot_positions << std::endl;
			os << "##### DESIRED VELOCITY #####" << std::endl;
			os << *this->desired_velocities << std::endl;
			RCLCPP_INFO(get_logger(), "%s", os.str().c_str());
		}
	};

	class SimulatedRobotInterface : public Modulo::Core::Cell
	{
	private:
		std::shared_ptr<StateRepresentation::JointState> robot_state;
		std::shared_ptr<StateRepresentation::JointState> desired_velocities;
		double dt;

	public:
		explicit SimulatedRobotInterface(const std::string & node_name, const std::chrono::milliseconds & period) :
		Cell(node_name, period),
		robot_state(std::make_shared<StateRepresentation::JointState>("robot", 6)),
		desired_velocities(std::make_shared<StateRepresentation::JointState>("robot", 6)),
		dt(0.001)
		{}

		bool on_configure()
		{
			this->robot_state->set_positions(Eigen::VectorXd::Random(6));
			this->add_subscription<sensor_msgs::msg::JointState>("/ds/desired_velocities", this->desired_velocities);
			this->add_publisher<sensor_msgs::msg::JointState>("/robot/joint_state", this->robot_state, 0);
			return true;
		}

		void step()
		{
			if(!this->desired_velocities->is_empty())
			{
				this->robot_state->set_positions((dt * *this->desired_velocities).get_velocities());
				this->robot_state->set_velocities(this->desired_velocities->get_velocities());
			}
		}
	};
}


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

	std::shared_ptr<LinearMotionGenerator> lmg = std::make_shared<LinearMotionGenerator>("linear_motion_generator", 1ms);
	std::shared_ptr<ConsoleVisualizer> cv = std::make_shared<ConsoleVisualizer>("console_visualizer", 100ms);
	std::shared_ptr<SimulatedRobotInterface> sri = std::make_shared<SimulatedRobotInterface>("simulated_robot_interface", 1ms);

	exe.add_node(lmg->get_node_base_interface());
	exe.add_node(cv->get_node_base_interface());
	exe.add_node(sri->get_node_base_interface());

	exe.spin();

	rclcpp::shutdown();

	return 0;
}