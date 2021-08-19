#include <dynamical_systems/Circular.hpp>
#include <dynamical_systems/Linear.hpp>
#include <eigen3/Eigen/Core>
#include <exception>
#include <iostream>
#include <rcutils/cmdline_parser.h>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/cartesian/CartesianTwist.hpp>

#include "modulo_core/Cell.hpp"

namespace {
class MotionGenerator : public modulo::core::Cell {
private:
  std::shared_ptr<state_representation::CartesianState> object_state;
  std::shared_ptr<state_representation::CartesianPose> current_pose;
  std::shared_ptr<state_representation::CartesianTwist> desired_twist;
  dynamical_systems::Circular motion_generator;

public:
  explicit MotionGenerator(const std::string& node_name, const std::chrono::milliseconds& period) : Cell(node_name, period),
                                                                                                    object_state(std::make_shared<state_representation::CartesianState>("object", "world")),
                                                                                                    current_pose(std::make_shared<state_representation::CartesianPose>("robot_test", "world")),
                                                                                                    desired_twist(std::make_shared<state_representation::CartesianTwist>("robot_test", "world")),
                                                                                                    motion_generator(state_representation::CartesianPose("robot_test_attractor", 0., 0., 0., "object")) {
    this->add_parameters(this->motion_generator.get_parameters());
  }

  bool on_configure() {
    this->add_subscription<geometry_msgs::msg::PoseStamped>("/object/pose", this->object_state);
    this->add_subscription<geometry_msgs::msg::TwistStamped>("/object/twist", this->object_state);
    this->add_subscription<geometry_msgs::msg::PoseStamped>("/robot_test/pose", this->current_pose);
    this->add_publisher<geometry_msgs::msg::TwistStamped>("/ds/desired_twist", this->desired_twist);
    return true;
  }

  void step() {
    if (!this->current_pose->is_empty()) {
      *this->desired_twist = this->motion_generator.evaluate(*this->current_pose);
    } else {
      this->desired_twist->initialize();
    }
    if (!this->object_state->is_empty()) {
      this->motion_generator.set_base_frame(*this->object_state);
    }
  }
};

class ConsoleVisualizer : public modulo::core::Cell {
private:
  std::shared_ptr<state_representation::CartesianPose> robot_pose;
  std::shared_ptr<state_representation::CartesianTwist> desired_twist;

public:
  explicit ConsoleVisualizer(const std::string& node_name, const std::chrono::milliseconds& period) : Cell(node_name, period),
                                                                                                      robot_pose(std::make_shared<state_representation::CartesianPose>("robot_test")),
                                                                                                      desired_twist(std::make_shared<state_representation::CartesianTwist>("robot_test")) {}

  bool on_configure() {
    this->add_subscription<geometry_msgs::msg::PoseStamped>("/robot_test/pose", this->robot_pose);
    this->add_subscription<geometry_msgs::msg::TwistStamped>("/ds/desired_twist", this->desired_twist);
    return true;
  }

  void step() {
    std::ostringstream os;
    os << std::endl;
    os << "##### ROBOT POSE #####" << std::endl;
    os << *this->robot_pose << std::endl;
    os << "##### DESIRED TWIST #####" << std::endl;
    os << *this->desired_twist << std::endl;
    RCLCPP_INFO(get_logger(), "%s", os.str().c_str());
  }
};

class SimulatedObject : public modulo::core::Cell {
private:
  std::shared_ptr<state_representation::CartesianPose> object_pose;
  std::shared_ptr<state_representation::CartesianTwist> object_twist;
  dynamical_systems::Linear<state_representation::CartesianState> motion_generator;
  std::chrono::milliseconds dt;
  double sign;

public:
  explicit SimulatedObject(const std::string& node_name, const std::chrono::milliseconds& period) : Cell(node_name, period),
                                                                                                    object_pose(std::make_shared<state_representation::CartesianPose>(state_representation::CartesianPose::Identity("object"))),
                                                                                                    object_twist(std::make_shared<state_representation::CartesianTwist>(state_representation::CartesianTwist("object"))),
                                                                                                    motion_generator(state_representation::CartesianPose("object_attractor", 2., 0., 0.), 1.0),
                                                                                                    dt(period),
                                                                                                    sign(-1) {
    this->add_parameters(this->motion_generator.get_parameters());
  }

  bool on_configure() {
    this->add_publisher<geometry_msgs::msg::PoseStamped>("/object/pose", this->object_pose);
    this->add_publisher<geometry_msgs::msg::TwistStamped>("/object/twist", this->object_twist);
    return true;
  }

  void step() {
    // compute the dynamics of the object
    *this->object_twist = this->motion_generator.evaluate(*this->object_pose);
    this->object_twist->set_angular_velocity(Eigen::Vector3d(0.2, 0., 0.));
    this->object_twist->clamp(0.5, 0.2);
    *this->object_pose = dt * *this->object_twist + *this->object_pose;

    // change attractor if previous was reached
    if (this->object_pose->dist(this->motion_generator.get_attractor()) < 1e-3) {
      this->set_parameter_value("attractor", state_representation::CartesianPose("object_attractor", sign * 2., 0., 0.));
      sign *= -1;
    }
    this->send_transform(*this->object_pose, "object");
  }
};

class SimulatedRobotInterface : public modulo::core::Cell {
private:
  std::shared_ptr<state_representation::CartesianPose> robot_pose;
  std::shared_ptr<state_representation::CartesianTwist> desired_twist;
  std::chrono::milliseconds dt;

public:
  explicit SimulatedRobotInterface(const std::string& node_name, const std::chrono::milliseconds& period) : Cell(node_name, period),
                                                                                                            robot_pose(std::make_shared<state_representation::CartesianPose>("robot_test", Eigen::Vector3d::Random(), Eigen::Quaterniond::UnitRandom())),
                                                                                                            desired_twist(std::make_shared<state_representation::CartesianTwist>("robot_test")),
                                                                                                            dt(period) {}

  bool on_configure() {
    this->add_subscription<geometry_msgs::msg::TwistStamped>("/ds/desired_twist", this->desired_twist);
    this->add_publisher<geometry_msgs::msg::PoseStamped>("/robot_test/pose", this->robot_pose);
    return true;
  }

  void step() {
    if (!this->desired_twist->is_empty()) {
      *this->robot_pose = dt * *this->desired_twist + *this->robot_pose;
    }
    this->send_transform(*this->robot_pose);
  }
};
}// namespace

/**
 * A lifecycle node has the same node API
 * as a regular node. This means we can spawn a
 * node, give it a name and add it to the executor.
 */
int main(int argc, char* argv[]) {
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exe;

  std::shared_ptr<MotionGenerator> lmg = std::make_shared<MotionGenerator>("motion_generator", 1ms);
  std::shared_ptr<ConsoleVisualizer> cv = std::make_shared<ConsoleVisualizer>("visualizer", 100ms);
  std::shared_ptr<SimulatedRobotInterface> sri = std::make_shared<SimulatedRobotInterface>("robot_interface", 1ms);
  std::shared_ptr<SimulatedObject> so = std::make_shared<SimulatedObject>("simulated_object", 1ms);

  exe.add_node(lmg->get_node_base_interface());
  exe.add_node(cv->get_node_base_interface());
  exe.add_node(sri->get_node_base_interface());
  exe.add_node(so->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return 0;
}