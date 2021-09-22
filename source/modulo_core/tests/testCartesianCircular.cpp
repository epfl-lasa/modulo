#include <dynamical_systems/Circular.hpp>
#include <eigen3/Eigen/Core>
#include <exception>
#include <iostream>
#include <rcutils/cmdline_parser.h>

#include "modulo_core/Cell.hpp"

namespace {
class MotionGenerator : public modulo::core::Cell {
private:
  std::shared_ptr<state_representation::CartesianPose> current_pose;
  std::shared_ptr<state_representation::CartesianTwist> desired_twist;
  dynamical_systems::Circular motion_generator;

public:
  explicit MotionGenerator(const std::string& node_name, const std::chrono::milliseconds& period) : Cell(node_name, period),
                                                                                                    current_pose(std::make_shared<state_representation::CartesianPose>("robot_test")),
                                                                                                    desired_twist(std::make_shared<state_representation::CartesianTwist>("robot_test")),
                                                                                                    motion_generator(state_representation::CartesianPose("robot_test", 0., 0., 0.)) {
    this->add_parameters(this->motion_generator.get_parameters());
  }

  bool on_configure() {
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

  exe.add_node(lmg->get_node_base_interface());
  exe.add_node(cv->get_node_base_interface());
  exe.add_node(sri->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return 0;
}