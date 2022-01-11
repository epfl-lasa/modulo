#include <eigen3/Eigen/Core>
#include <exception>
#include <iostream>

#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/cartesian/CartesianTwist.hpp>
#include <dynamical_systems/DynamicalSystemFactory.hpp>

#include "modulo_core/Cell.hpp"
#include "modulo_core/Component.hpp"

using namespace state_representation;
using namespace dynamical_systems;

namespace {
class LinearMotionGenerator : public modulo::core::Cell {
private:
  std::shared_ptr<CartesianPose> current_pose;
  std::shared_ptr<CartesianTwist> desired_twist;
  std::shared_ptr<IDynamicalSystem<CartesianState>> motion_generator;

public:
  explicit LinearMotionGenerator(const std::string& node_name, const std::chrono::milliseconds& period) :
      Cell(node_name, period),
      current_pose(std::make_shared<CartesianPose>("robot_test")),
      desired_twist(std::make_shared<CartesianTwist>("robot_test")),
      motion_generator(
          DynamicalSystemFactory<CartesianState>::create_dynamical_system(
              DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM::POINT_ATTRACTOR
          )) {
    motion_generator->set_parameter(make_shared_parameter("attractor", CartesianPose::Random("robot_test")));
  }

  bool on_configure() {
    this->add_state_subscription("/robot_test/pose", this->current_pose);
    this->add_state_publisher("/ds/desired_twist", this->desired_twist);
    return true;
  }

  void step() {
    if (!this->current_pose->is_empty()) {
      *this->desired_twist = this->motion_generator->evaluate(*this->current_pose);
      // change attractor if previous was reached
      if (this->current_pose->dist(this->motion_generator->get_parameter_value<CartesianPose>("attractor")) < 1e-3) {
        this->set_parameter_value("attractor", CartesianPose::Random("robot_test"));
      }
    } else {
      this->desired_twist->initialize();
    }
    this->send_transform(this->motion_generator->get_parameter_value<CartesianPose>("attractor"), "attractor");
  }
};

class ConsoleVisualizer : public modulo::core::Component {
private:
  std::shared_ptr<CartesianPose> robot_pose;
  std::shared_ptr<CartesianTwist> desired_twist;

public:
  explicit ConsoleVisualizer(const std::string& node_name, const std::chrono::milliseconds& period) :
      Component(node_name, period),
      robot_pose(std::make_shared<CartesianPose>("robot_test")),
      desired_twist(std::make_shared<CartesianTwist>("robot_test")) {}

  bool on_configure() {
    this->add_state_subscription("/robot_test/pose", this->robot_pose);
    this->add_state_subscription("/ds/desired_twist", this->desired_twist);
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
  std::shared_ptr<CartesianPose> robot_pose;
  std::shared_ptr<CartesianTwist> desired_twist;
  std::chrono::milliseconds dt;

public:
  explicit SimulatedRobotInterface(const std::string& node_name, const std::chrono::milliseconds& period) :
      Cell(node_name, period),
      robot_pose(
          std::make_shared<CartesianPose>(
              "robot_test", Eigen::Vector3d(1.18, 0, 1.6), Eigen::Quaterniond(0.73, 0, 0.68, 0))),
      desired_twist(std::make_shared<CartesianTwist>("robot_test")),
      dt(period) {}

  bool on_configure() {
    this->add_state_subscription("/ds/desired_twist", this->desired_twist);
    this->add_state_publisher("/robot_test/pose", this->robot_pose);
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

  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<LinearMotionGenerator> lmg = std::make_shared<LinearMotionGenerator>("motion_generator", 1ms);
  std::shared_ptr<ConsoleVisualizer> cv = std::make_shared<ConsoleVisualizer>("visualizer", 100ms);
  std::shared_ptr<SimulatedRobotInterface> sri = std::make_shared<SimulatedRobotInterface>("robot_interface", 1ms);

  exe.add_node(lmg->get_node_base_interface());
  exe.add_node(cv->get_node_base_interface());
  exe.add_node(sri->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return 0;
}