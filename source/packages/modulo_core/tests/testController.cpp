#include "controllers/impedance/Dissipative.hpp"
#include "dynamical_systems/Linear.hpp"
#include "modulo_core/Cell.hpp"
#include "modulo_msgs/srv/follow_path.hpp"
#include "rcutils/cmdline_parser.h"
#include "robot_model/Model.hpp"
#include <exception>
#include <iostream>

using namespace StateRepresentation;

namespace {
class LinearMotionGenerator : public modulo::core::Cell {
private:
  std::shared_ptr<Parameter<CartesianPose>> home_pose_;      ///< home pose parameter
  std::shared_ptr<Parameter<double>> distance_tolerance_;    ///< distance tolerance to go to the next attractor
  CartesianPose current_pose_;                               ///< current pose of the end-effector
  std::shared_ptr<CartesianTwist> desired_twist_;            ///< desired twist of the end-effector
  DynamicalSystems::Linear<CartesianState> motion_generator_;///< motion generator as a linear DS

  void play_trajectory(const std::shared_ptr<rmw_request_id_t> request_header,
                       const std::shared_ptr<modulo_msgs::srv::FollowPath::Request> request,
                       std::shared_ptr<modulo_msgs::srv::FollowPath::Response> response) {
    (void) request_header;
    // start looping through the points
    for (auto& p : request->trajectory.poses) {
      // convert the PoseStamped to a CartesianPose
      CartesianPose pose(this->current_pose_.get_name(), this->current_pose_.get_reference_frame());
      modulo::core::communication::state_conversion::read_msg(pose, p);
      // set the point as attractor to the ds
      this->motion_generator_.set_attractor(pose);
      // change attractor when close enough but not too close to
      // not have 0 velocity
      while (this->current_pose_.dist(this->motion_generator_.get_attractor()) > this->distance_tolerance_->get_value()) {
        // sleep for a small perdiod of time
        std::this_thread::sleep_for(this->get_period());
      }
    }
    // return success
    response->success = true;
  }

public:
  explicit LinearMotionGenerator(const std::string& node_name, const std::chrono::milliseconds& period) : Cell(node_name, period),
                                                                                                          home_pose_(std::make_shared<Parameter<CartesianPose>>("home_pose", CartesianPose("iiwa_link_ee", 0.5, 0., 0.75))),
                                                                                                          distance_tolerance_(std::make_shared<Parameter<double>>("distance_tolerance", 0.01)),
                                                                                                          current_pose_(home_pose_->get_value()),
                                                                                                          desired_twist_(std::make_shared<CartesianTwist>("iiwa_link_ee")),
                                                                                                          motion_generator_(home_pose_->get_value(), 1.0) {
    this->add_parameter(home_pose_);
    this->add_parameter(distance_tolerance_);
    this->create_service<modulo_msgs::srv::FollowPath>("~/play_trajectory",
                                                       std::bind(&LinearMotionGenerator::play_trajectory,
                                                                 this,
                                                                 std::placeholders::_1,
                                                                 std::placeholders::_2,
                                                                 std::placeholders::_3));
  }

  bool on_configure() {
    this->add_publisher<geometry_msgs::msg::TwistStamped>("/ds/desired_twist", this->desired_twist_);
    return true;
  }

  void step() {
    try {
      // get the eef tranform
      this->current_pose_ = this->lookup_transform(this->current_pose_.get_name(),
                                                   this->current_pose_.get_reference_frame());
      // compute the desired twist based on the attractor
      *this->desired_twist_ = this->motion_generator_.evaluate(this->current_pose_);
    } catch (tf2::LookupException& ex) {
      RCLCPP_ERROR(get_logger(), "%s", ex.what());
      *this->desired_twist_ = CartesianTwist::Zero("iiwa_link_ee");
    }
  }
};

class RobotInterface : public modulo::core::Cell {
private:
  std::shared_ptr<Parameter<bool>> compliant_mode_;                                    ///< parameter to turn on and off the compliance
  std::shared_ptr<StateRepresentation::CartesianTwist> desired_twist_;                 ///< desired twist of the end-effector
  std::shared_ptr<StateRepresentation::JointState> current_robot_state_;               ///< the current state read from the robot
  std::shared_ptr<StateRepresentation::JointTorques> torques_command_;                 ///< the desired torque command to send
  controllers::impedance::Dissipative<StateRepresentation::CartesianState> controller_;///< the controller (PassiveDS)
  RobotModel::Model iiwa_model_;                                                       ///< the model of the robot

public:
  explicit RobotInterface(const std::string& node_name, const std::chrono::milliseconds& period) : Cell(node_name, period),
                                                                                                   compliant_mode_(std::make_shared<Parameter<bool>>("compliant_mode", false)),
                                                                                                   desired_twist_(std::make_shared<StateRepresentation::CartesianTwist>(StateRepresentation::CartesianTwist::Zero("iiwa_link_ee"))),
                                                                                                   current_robot_state_(std::make_shared<StateRepresentation::JointState>(StateRepresentation::JointState::Zero("iiwa", 7))),
                                                                                                   torques_command_(std::make_shared<StateRepresentation::JointTorques>(StateRepresentation::JointTorques::Zero("iiwa", 7))),
                                                                                                   iiwa_model_("iiwa", std::string(TEST_FIXTURES) + "/iiwa7.urdf") {
    this->add_parameter(this->compliant_mode_);
    this->add_parameters(this->controller_.get_parameters());
  }

  bool on_configure() {
    this->add_subscription<geometry_msgs::msg::TwistStamped>("/ds/desired_twist", this->desired_twist_);
    this->add_subscription<sensor_msgs::msg::JointState>("/iiwa/joint_states", this->current_robot_state_);
    this->add_publisher<std_msgs::msg::Float64MultiArray>("/iiwa/TorqueController/command", this->torques_command_);
    return true;
  }

  void step() {
    // send a zero torque by default
    *this->torques_command_ = StateRepresentation::JointTorques::Zero("iiwa", 7);
    // compute the command if we received both the current state and the desired twist and the robot is not in compliant mode
    if (!this->current_robot_state_->is_empty() && !this->desired_twist_->is_empty() && !this->compliant_mode_->get_value()) {
      // use the model forward kinematic to extract eef CartesianTwist
      StateRepresentation::Jacobian jacobian = this->iiwa_model_.compute_jacobian(*this->current_robot_state_);
      // compute current twist
      StateRepresentation::CartesianTwist current_twist = jacobian * static_cast<StateRepresentation::JointVelocities>(*this->current_robot_state_);
      // change twist name and reference frame
      current_twist.set_name(this->desired_twist_->get_name());
      current_twist.set_reference_frame(this->desired_twist_->get_reference_frame());
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

  std::shared_ptr<LinearMotionGenerator> lmg = std::make_shared<LinearMotionGenerator>("motion_generator", 5ms);
  std::shared_ptr<RobotInterface> sri = std::make_shared<RobotInterface>("robot_interface", 5ms);

  exe.add_node(lmg->get_node_base_interface());
  exe.add_node(sri->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return 0;
}