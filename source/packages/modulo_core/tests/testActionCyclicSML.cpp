#include "modulo_core/Action.hpp"
#include "modulo_core/Monitor.hpp"
#include "modulo_core/Visualizer.hpp"
#include "dynamical_systems/Linear.hpp"
#include "rcutils/cmdline_parser.h"
#include <eigen3/Eigen/Core>
#include <iostream>
#include <exception>
#include <boost/sml.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

int e = 10;
namespace sml = boost::sml;


namespace {

	struct e_start {};
	struct e_launch {};
	struct terminated {};
	struct interrupted {}; 
}

class MoveAction : public Modulo::Actions::Action<StateRepresentation::CartesianState>
{
private:
	std::shared_ptr<StateRepresentation::CartesianPose> target_pose;

public:
	template <typename DurationT>
	explicit MoveAction(const std::string& node_name, const std::chrono::duration<int64_t, DurationT>& period) :
	Action<StateRepresentation::CartesianState>(std::make_shared<StateRepresentation::CartesianPose>("robot_test"), std::make_shared<StateRepresentation::CartesianTwist>("robot_test"), node_name, period, false),
	target_pose(std::make_shared<StateRepresentation::CartesianPose>("attractor", Eigen::Vector3d::Random()))
	{}

	bool on_configure()
	{
		this->add_subscription<geometry_msgs::msg::PoseStamped>("/robot_test/pose", this->get_input_state(), 0);
		this->add_publisher<geometry_msgs::msg::TwistStamped>("/ds/desired_twist", this->get_output_state());

		std::shared_ptr<DynamicalSystems::Linear<StateRepresentation::CartesianState> > move_dynamic = std::make_shared<DynamicalSystems::Linear<StateRepresentation::CartesianState> >(1);
		move_dynamic->set_attractor(this->target_pose);
		this->set_dynamic(move_dynamic);
		return true;
	}

	bool is_terminated()
	{
		return this->get_input_state()->dist(*this->target_pose) < 1e-3;
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

	bool on_configure()
	{
		this->add_publisher<geometry_msgs::msg::PoseStamped>("/ds/attractor", target_pose, 0);
		this->add_asynchronous_transform_broadcaster(target_pose, 1ms);
		return true;
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

template <typename StateMachineT>
class EventHandler : public Modulo::Core::Cell
{
	private:
		std::shared_ptr<StateMachineT> sm;

		void topic_callback(const std_msgs::msg::Int64::SharedPtr msg) const
	    {
	      e =  msg->data;
	    }
	    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscription_;

	public:
		explicit EventHandler(std::shared_ptr<StateMachineT> sent_sm):
		Cell("event_handler", 100ms, false)
		{
			this->sm = sent_sm;
		}

		bool on_configure()
		{
			this->subscription_ = this->create_subscription<std_msgs::msg::Int64>("/event_id", 10, std::bind(&EventHandler::topic_callback, this, std::placeholders::_1));
			return true;
		}

		void step()
		{
			//this->sm->visit_current_states([](auto state) { std::cout << "Current state: " << state.c_str() << std::endl; });

		    switch(e)
		    {
		    	case 0:
		    	  this->sm->process_event(e_start{});
		          break;
		        case 1:
		          this->sm->process_event(e_launch{});
		          break;
		        case 2:
		          this->sm->process_event(terminated{});
		          break;
		        case 3:
		          this->sm->process_event(interrupted{});
		          break;
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
	template <typename DurationT>
	explicit ConsoleVisualizer(const std::string& node_name, const std::chrono::duration<int64_t, DurationT>& period) :
	Visualizer(node_name, period, false),
	robot_pose(std::make_shared<StateRepresentation::CartesianPose>("robot_test")),
	desired_twist(std::make_shared<StateRepresentation::CartesianTwist>("robot_test"))
	{}

	bool on_configure()
	{
		this->add_subscription<geometry_msgs::msg::PoseStamped>("/robot_test/pose", this->robot_pose);
		this->add_subscription<geometry_msgs::msg::TwistStamped>("/ds/desired_twist", this->desired_twist);
		return true;
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

	bool on_configure()
	{
		this->add_subscription<geometry_msgs::msg::TwistStamped>("/ds/desired_twist", this->desired_twist, 0);
		this->add_publisher<geometry_msgs::msg::PoseStamped>("/robot_test/pose", this->robot_pose, 0);
		return true;
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


template <class R, class... Ts>
auto call_impl(R (*f)(Ts...), Ts... args) {
  return [f, args...]() {  f(args...); };
}

/**
 * Simple wrapper to call free/member functions
 * @param args function, [optional] this
 * @return function(args...)
 */
auto call = [](auto... args) { return call_impl(args...); };

//void switch_states(std::shared_ptr<MoveAction> state1, std::shared_ptr<MoveAction> state2)
void switch_states(int a, int b)
{ 
	std::cout << "Action switch_states" << std::endl;
	// To be done: throw exception if node to be activated is not in correct state
	//state1->deactivate();
	//state2->activate();
	std::cout << "a" << a << "b" << b << std::endl;
}

void start()
{ 
	std::cout << "Starting" << std::endl;
}

struct state_machine
{
	auto operator()() const noexcept {
	  using namespace sml;
	  return make_transition_table(

	  	//*"move_home"_s + sml::on_entry<_> / activate = "move_home"_s,
	  	//*"move_home"_s + event<start> / activate = "move_home"_s,
	  	//*"move_home"_s + event<start> / call(this, &state_machine::switch_states) = "move_action1"_s
	  	*"idle"_s + event<e_start> / call(start) = "move_home"_s,
	  	"move_home"_s + event<e_launch> / call(switch_states, 1, 2) = "move_action1"_s
	  	//"move_action1"_s + event<terminated> / call(this, &state_machine::switch_states) = "move_action2"_s,
	  	// "move_action2"_s + event<terminated> / switch_states(0) = "move_action1"_s,
	  	// "move_action1"_s + event<interrupted> / switch_states(0) = "move_home"_s,
	  	// "move_action2"_s + event<interrupted> / switch_states(0) = "move_home"_s
	  );
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

	std::shared_ptr<MoveAction> mh = std::make_shared<MoveAction>("move_home", 250us);
	std::shared_ptr<MoveAction> ma1 = std::make_shared<MoveAction>("move_action1", 250us);
	std::shared_ptr<MoveAction> ma2 = std::make_shared<MoveAction>("move_action2", 250us);
	std::shared_ptr<SimulatedRobotInterface> sri = std::make_shared<SimulatedRobotInterface>("robot_interface", 250us);

	exe.add_node(mh->get_node_base_interface());
	exe.add_node(ma1->get_node_base_interface());
	exe.add_node(ma2->get_node_base_interface());
	exe.add_node(sri->get_node_base_interface());
	
	std::list<std::string> monitored_nodes = {"move_home", "move_action1", "move_action2", "robot_interface"};
	std::shared_ptr<Modulo::Monitors::Monitor> mo = std::make_shared<Modulo::Monitors::Monitor>("monitor", monitored_nodes, 1s);
	exe.add_node(mo->get_node_base_interface());

	
	//sml::sm<state_machine> sm;
	std::shared_ptr<sml::sm<state_machine>> sm = std::make_shared<sml::sm<state_machine>>();
	std::shared_ptr<EventHandler<sml::sm<state_machine>>> eh = std::make_shared<EventHandler<sml::sm<state_machine>>>(sm);
	exe.add_node(eh->get_node_base_interface());


    exe.spin();
    rclcpp::shutdown();
	return 0;
}