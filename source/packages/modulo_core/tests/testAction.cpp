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
	template <typename DurationT>
	explicit MoveAction(const std::string& node_name, const std::chrono::duration<int64_t, DurationT>& period) :
	Action<StateRepresentation::CartesianState>(std::make_shared<StateRepresentation::CartesianPose>("robot_test"), std::make_shared<StateRepresentation::CartesianTwist>("robot_test"), node_name, period, false),
	target_pose(std::make_shared<StateRepresentation::CartesianPose>("attractor", Eigen::Vector3d::Zero()))
	{}

	void on_configure()
	{
		this->add_subscription<geometry_msgs::msg::PoseStamped>("/robot_test/pose", this->get_input_state(), 0);
		this->add_subscription<geometry_msgs::msg::PoseStamped>("/ds/attractor", this->target_pose, 0);
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
	unsigned int counter;

public:
	template <typename DurationT>
	explicit RandomAttractor(const std::string& node_name, const std::chrono::duration<int64_t, DurationT>& period):
	Cell(node_name, period, false),
	target_pose(std::make_shared<StateRepresentation::CartesianPose>("attractor", Eigen::Vector3d::Random(), Eigen::Quaterniond::UnitRandom())),
	counter(0)
	{}

	void on_configure()
	{
		this->add_publisher<geometry_msgs::msg::PoseStamped>("/ds/attractor", target_pose, 0);
		this->add_asynchronous_transform_broadcaster(target_pose, 100ms);
	}

	void step()
	{
		this->target_pose->set_pose(Eigen::Vector3d::Random(), Eigen::Quaterniond::UnitRandom());
		/*if (counter == 5)
		{
			counter = 0;
			this->deactivate();
		}
		else
		{
			++counter;
		}*/
	}
};

class ConsoleVisualizer : public Modulo::Visualizers::Visualizer
{
private:
	std::shared_ptr<StateRepresentation::CartesianPose> robot_pose;
	std::shared_ptr<StateRepresentation::CartesianTwist> desired_twist;
	std::shared_ptr<StateRepresentation::Parameter<double> > ds_gain;

public:
	template <typename DurationT>
	explicit ConsoleVisualizer(const std::string& node_name, const std::chrono::duration<int64_t, DurationT>& period) :
	Visualizer(node_name, period, false),
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
	std::chrono::nanoseconds dt;

public:
	template <typename DurationT>
	explicit SimulatedRobotInterface(const std::string& node_name, const std::chrono::duration<int64_t, DurationT>& period):
	Cell(node_name, period, false),
	robot_pose(std::make_shared<StateRepresentation::CartesianPose>("robot_test", Eigen::Vector3d::Random(), Eigen::Quaterniond::UnitRandom())),
	desired_twist(std::make_shared<StateRepresentation::CartesianTwist>("robot_test")),
	dt(period)
	{}

	void on_configure()
	{
		this->add_subscription<geometry_msgs::msg::TwistStamped>("/ds/desired_twist", this->desired_twist, 0);
		this->add_publisher<geometry_msgs::msg::PoseStamped>("/robot_test/pose", this->robot_pose, 0);
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

	rclcpp::executors::MultiThreadedExecutor exe;
	std::shared_ptr<MoveAction> ma = std::make_shared<MoveAction>("move_action", 100ms);
	std::shared_ptr<RandomAttractor> ra = std::make_shared<RandomAttractor>("random_attractor", 5s);
	std::shared_ptr<SimulatedRobotInterface> sri = std::make_shared<SimulatedRobotInterface>("robot_interface", 100ms);

	exe.add_node(ma->get_node_base_interface());
	exe.add_node(ra->get_node_base_interface());
	exe.add_node(sri->get_node_base_interface());

	std::list<std::string> monitored_nodes = {"move_action", "random_attractor", "robot_interface"};
	std::shared_ptr<Modulo::Monitors::Monitor> mo = std::make_shared<Modulo::Monitors::Monitor>("monitor", monitored_nodes, 1s);
	exe.add_node(mo->get_node_base_interface());

	exe.spin();

	rclcpp::shutdown();

	return 0;
}