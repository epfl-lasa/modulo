#include "controllers/impedance/Dissipative.hpp"
#include "dynamical_systems/Linear.hpp"
#include "modulo_core/Cell.hpp"
#include "rcutils/cmdline_parser.h"
#include "robot_model/Model.hpp"
#include <exception>
#include <iostream>

using namespace StateRepresentation;

namespace {
class LinearMotionGenerator : public modulo::core::Cell {
private:
  std::shared_ptr<CartesianPose> current_pose;
  std::shared_ptr<CartesianTwist> desired_twist;
  std::shared_ptr<Trajectory<CartesianState>> desired_trajecory;
  DynamicalSystems::Linear<CartesianState> motion_generator;

public:
  explicit LinearMotionGenerator(const std::string& node_name, const std::chrono::milliseconds& period) : Cell(node_name, period),
                                                                                                          current_pose(std::make_shared<StateRepresentation::CartesianPose>("robot_test")),
                                                                                                          desired_twist(std::make_shared<StateRepresentation::CartesianTwist>("robot_test")),
                                                                                                          motion_generator(StateRepresentation::CartesianPose::Random("robot_test"), 1.0) {
    this->add_parameters(this->motion_generator.get_parameters());
  }

  bool on_configure() {
    this->add_publisher<geometry_msgs::msg::TwistStamped>("/ds/desired_twist", this->desired_twist);
    return true;
  }

  void step() {
    // get the eef tranform

    // compute the desired twist
    *this->desired_twist = this->motion_generator.evaluate(*this->current_pose);
  }
};

class RobotInterface : public modulo::core::Cell {
private:
  std::shared_ptr<StateRepresentation::CartesianPose> robot_pose_;
  std::shared_ptr<StateRepresentation::CartesianTwist> desired_twist_;
  std::shared_ptr<StateRepresentation::JointState> current_robot_state_;
  std::shared_ptr<StateRepresentation::JointTorques> torques_command_;
  controllers::impedance::Dissipative<StateRepresentation::CartesianState> controller_;
  RobotModel::Model iiwa_model_;
  std::chrono::milliseconds dt_;

public:
  explicit RobotInterface(const std::string& node_name, const std::chrono::milliseconds& period) : Cell(node_name, period),
                                                                                                   robot_pose_(std::make_shared<StateRepresentation::CartesianPose>("robot_test", Eigen::Vector3d(1.18, 0, 1.6), Eigen::Quaterniond(0.73, 0, 0.68, 0))),
                                                                                                   desired_twist_(std::make_shared<StateRepresentation::CartesianTwist>(StateRepresentation::CartesianTwist::Zero("robot_test"))),
                                                                                                   current_robot_state_(std::make_shared<StateRepresentation::JointState>(StateRepresentation::JointState::Zero("iiwa", 7))),
                                                                                                   torques_command_(std::make_shared<StateRepresentation::JointTorques>(StateRepresentation::JointTorques::Zero("iiwa", 7))),
                                                                                                   iiwa_model_("iiwa", std::string(TEST_FIXTURES) + "/iiwa7.urdf"),
                                                                                                   dt_(period) {}

  bool on_configure() {
    this->add_subscription<geometry_msgs::msg::TwistStamped>("/ds/desired_twist", this->desired_twist_);
    this->add_subscription<sensor_msgs::msg::JointState>("robot/joints", this->current_robot_state_);
    this->add_publisher<sensor_msgs::msg::JointState>("robot/command", this->torques_command_);
    return true;
  }

  void step() {
    if (!this->current_robot_state_->is_empty() && !this->desired_twist_->is_empty()) {
      // use the model forward kinematic to extract eef CartesianTwist
      StateRepresentation::Jacobian jacobian = this->iiwa_model_.compute_jacobian(*this->current_robot_state_);
      // compute current twist
      StateRepresentation::CartesianTwist current_twist = jacobian * static_cast<StateRepresentation::JointVelocities>(*this->current_robot_state_);
      //get the wrench command from the controller
      *this->torques_command_ = this->controller_.compute_command(*this->desired_twist_, current_twist, jacobian);
    }
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
  std::shared_ptr<RobotInterface> sri = std::make_shared<RobotInterface>("robot_interface", 1ms);

  exe.add_node(lmg->get_node_base_interface());
  exe.add_node(sri->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return 0;
}