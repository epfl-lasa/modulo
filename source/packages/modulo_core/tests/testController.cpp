#include "modulo_core/Cell.hpp"
#include "modulo_msgs/action/follow_path.hpp"
#include <controllers/impedance/Dissipative.hpp>
#include <dynamical_systems/Linear.hpp>
#include <exception>
#include <functional>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcutils/cmdline_parser.h>
#include <robot_model/Model.hpp>
#include <thread>

using namespace StateRepresentation;

namespace {


class LinearMotionGenerator : public modulo::core::Cell {
private:
  using FollowPath = modulo_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<FollowPath>;

  rclcpp_action::Server<FollowPath>::SharedPtr action_server_;///< shared_ptr to the action server
  std::shared_ptr<Parameter<CartesianPose>> attractor_pose_;       ///< home pose parameter
  std::shared_ptr<Parameter<double>> distance_tolerance_;     ///< distance tolerance to go to the next attractor
  CartesianPose current_pose_;                                ///< current pose of the end-effector
  std::shared_ptr<CartesianTwist> desired_twist_;             ///< desired twist of the end-effector
  DynamicalSystems::Linear<CartesianState> motion_generator_; ///< motion generator as a linear DS
  std::shared_ptr<Parameter<std::vector<double>>> clamping_values_;
  bool last_action_attractor_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID&,
      std::shared_ptr<const FollowPath::Goal>) {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleFollowPath>) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFollowPath> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&LinearMotionGenerator::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFollowPath> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    this->last_action_attractor_ = false;
    double old_tolerance = distance_tolerance_->get_value();
    // increase the tolerance
    this->set_parameter_value("distance_tolerance", 0.1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<FollowPath::Feedback>();
    auto result = std::make_shared<FollowPath::Result>();
    // start looping through the points
    unsigned int nb_points = goal->path.poses.size();
    for (unsigned int i = 0; (i < nb_points) && rclcpp::ok(); ++i) {
      if (i == nb_points - 1) {
        this->last_action_attractor_ = true;
        this->set_parameter_value("distance_tolerance", old_tolerance);
      }
      // convert the PoseStamped to a CartesianPose
      CartesianPose pose(this->current_pose_.get_name(), this->current_pose_.get_reference_frame());
      modulo::core::communication::state_conversion::read_msg(pose, goal->path.poses[i]);
      this->set_parameter_value("attractor_pose", pose);
      while (pose.dist(this->motion_generator_.get_attractor(), CartesianStateVariable::POSE) > this->distance_tolerance_->get_value()) {
        std::this_thread::sleep_for(this->get_period());
        RCLCPP_INFO(this->get_logger(), "Waiting for attractor change");
      }
      // publish feedback
      feedback->percentage_of_completion = i / nb_points * 100;
      goal_handle->publish_feedback(feedback);
      // change attractor when close enough but not too close to
      // not have 0 velocity
      bool attractor_not_reached;
      do {
        attractor_not_reached = (this->current_pose_.is_empty() || this->current_pose_.dist(this->motion_generator_.get_attractor(), CartesianStateVariable::POSE) > this->distance_tolerance_->get_value());
        // sleep for a small perdiod of time
        std::this_thread::sleep_for(10 * this->get_period());
        // Check if there is a cancel request
        if (goal_handle->is_canceling()) {
          this->last_action_attractor_ = true;
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          return;
        }
      } while (attractor_not_reached && rclcpp::ok());
    }
    // Check if goal is done
    if (rclcpp::ok()) {
      this->last_action_attractor_ = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

public:
  explicit LinearMotionGenerator(const std::string& node_name, const std::chrono::milliseconds& period) : Cell(node_name, period),
                                                                                                          attractor_pose_(std::make_shared<Parameter<CartesianPose>>("attractor_pose", CartesianPose("iiwa_link_ee_polish",
                                                                                                          Eigen::Vector3d(0.48, 0.0, 0.70),
                                                                                                          Eigen::Quaterniond( 0.37, 0.135,  0.889, 0.175),
                                                                                                          "iiwa_link_0"))),
                                                                                                          distance_tolerance_(std::make_shared<Parameter<double>>("distance_tolerance", 0.01)),
                                                                                                          current_pose_(attractor_pose_->get_value()),
                                                                                                          desired_twist_(std::make_shared<CartesianTwist>("iiwa_link_ee_polish", "iiwa_link_0")),
                                                                                                          motion_generator_(attractor_pose_->get_value(), std::vector<double> {20.0, 20.0, 20.0, 8.0, 8.0, 8.0}),
                                                                                                          clamping_values_(std::make_shared<Parameter<std::vector<double>>>("clamping_values", std::vector<double>{0.25, 0.5, 0.005, 0.005})),
                                                                                                          last_action_attractor_(true) {
    using namespace std::placeholders;

    this->add_parameter(attractor_pose_);
    this->add_parameter(distance_tolerance_);
    this->add_parameter(clamping_values_);
    this->action_server_ = rclcpp_action::create_server<modulo_msgs::action::FollowPath>(
        this,
        "~/follow_path",
        std::bind(&LinearMotionGenerator::handle_goal, this, _1, _2),
        std::bind(&LinearMotionGenerator::handle_cancel, this, _1),
        std::bind(&LinearMotionGenerator::handle_accepted, this, _1));
  }

  bool on_configure() {
    this->last_action_attractor_ = true;
    this->add_publisher<geometry_msgs::msg::TwistStamped>("/ds/desired_twist", this->desired_twist_);
    return true;
  }

  void step() {
    try {
      // get the eef tranform
      this->motion_generator_.set_attractor(this->attractor_pose_->get_value());
      this->current_pose_ = this->lookup_transform(this->current_pose_.get_name(),
                                                   "iiwa_link_0");
      // compute the desired twist based on the attractor
      *this->desired_twist_ = this->motion_generator_.evaluate(this->current_pose_);
      std::vector<double> clamp_values = this->clamping_values_->get_value();
      this->desired_twist_->clamp(clamp_values[0], clamp_values[1], clamp_values[2], clamp_values[3]);
      this->send_transform(this->attractor_pose_->get_value(), "attractor");
      // if we are not in the last attractor we normalize the twist
      if (!this->last_action_attractor_) {
        this->desired_twist_->normalize();
        Eigen::VectorXd gain(6);
        gain << clamp_values[0], clamp_values[0], clamp_values[0],
                clamp_values[1], clamp_values[1], clamp_values[1];
        *this->desired_twist_ *= gain.asDiagonal();
      }
    } catch (tf2::LookupException& ex) {
      RCLCPP_ERROR(get_logger(), "%s", ex.what());
      *this->desired_twist_ = CartesianTwist::Zero("iiwa_link_ee_polish", "iiwa_link_0");
      this->current_pose_.initialize();
    }
  }
};

class FTSensorInterface : public modulo::core::Cell {
private:
  std::shared_ptr<StateRepresentation::CartesianWrench> raw_sensor_data_;
  std::shared_ptr<StateRepresentation::CartesianWrench> calibrated_sensor_data_;
  std::shared_ptr<StateRepresentation::CartesianPose> ft_in_robot_;
  std::shared_ptr<StateRepresentation::CartesianPose> tool_in_ft_;
  StateRepresentation::CartesianWrench force_compensation_;

public:
  explicit FTSensorInterface(const std::string& node_name, const std::chrono::milliseconds& period) : 
    Cell(node_name, period),
    raw_sensor_data_(std::make_shared<StateRepresentation::CartesianWrench>("iiwa_link_ee", "iiwa_link_ee")),
    calibrated_sensor_data_(std::make_shared<StateRepresentation::CartesianWrench>("iiwa_link_ee_polish", "iiwa_link_0")),
    ft_in_robot_(std::make_shared<StateRepresentation::CartesianPose>("iiwa_link_ee", "iiwa_link_0")),
    tool_in_ft_(std::make_shared<StateRepresentation::CartesianPose>("iiwa_link_ee_polish", "iiwa_link_ee")),
    force_compensation_(StateRepresentation::CartesianWrench("iiwa_link_ee_polish", Eigen::Vector3d(0., 0., -7.55), "iiwa_link_0")) {}

    bool on_configure() {
      this->add_subscription<geometry_msgs::msg::WrenchStamped>("/ft_sensor/netft_data", this->raw_sensor_data_);
      this->add_subscription<geometry_msgs::msg::Pose>("/iiwa/EndEffectorPosition", this->ft_in_robot_);
      this->add_subscription<geometry_msgs::msg::Pose>("/iiwa/EE2ToolPosition", this->tool_in_ft_);
      this->add_publisher<geometry_msgs::msg::WrenchStamped>("/calibrated_ft_sensor/iiwa_link_ee_polish", this->calibrated_sensor_data_);
      return true;
    }

    void step() {
      if (!this->raw_sensor_data_->is_empty() && !this->ft_in_robot_->is_empty() && !this->tool_in_ft_->is_empty()) {
        // std::cout<<*this->raw_sensor_data_<<std::endl;
        StateRepresentation::CartesianWrench force_at_tooltip("iiwa_link_ee_polish", "iiwa_link_0");
        force_at_tooltip.set_force(*this->ft_in_robot_ * this->raw_sensor_data_->get_force());
        *this->calibrated_sensor_data_ = force_at_tooltip - force_compensation_;
      }
    }
};

class RobotInterface : public modulo::core::Cell {
private:
  std::shared_ptr<StateRepresentation::Parameter<bool>> compliant_mode_;                                    ///< parameter to turn on and off the compliance
  std::shared_ptr<StateRepresentation::Parameter<double>> desired_radial_force_;
  std::shared_ptr<StateRepresentation::CartesianState> desired_state_;                 ///< desired twist of the end-effector
  std::shared_ptr<StateRepresentation::CartesianState> feedback_state_;                 ///< current cartesian state of the robot
  std::shared_ptr<StateRepresentation::JointState> current_robot_state_;               ///< the current state read from the robot
  std::shared_ptr<StateRepresentation::JointTorques> torques_command_;                 ///< the desired torque command to send
  controllers::impedance::Dissipative<StateRepresentation::CartesianState> controller_;///< the controller (PassiveDS)
  RobotModel::Model iiwa_model_;                                                       ///< the model of the robot

public:
  explicit RobotInterface(const std::string& node_name, const std::chrono::milliseconds& period) : Cell(node_name, period),
                                                                                                   compliant_mode_(std::make_shared<Parameter<bool>>("compliant_mode", false)),
                                                                                                   desired_radial_force_(std::make_shared<Parameter<double>>("desired_radial_force", 0.0)),
                                                                                                   desired_state_(std::make_shared<StateRepresentation::CartesianState>(StateRepresentation::CartesianState::Identity("iiwa_link_ee_polish", "iiwa_link_0"))),
                                                                                                   feedback_state_(std::make_shared<StateRepresentation::CartesianState>(StateRepresentation::CartesianState::Identity("iiwa_link_ee_polish", "iiwa_link_0"))),
                                                                                                   iiwa_model_("iiwa", std::string(TEST_FIXTURES) + "/iiwa7.urdf") {

    this->current_robot_state_ = std::make_shared<StateRepresentation::JointState>(StateRepresentation::JointState::Zero("iiwa", iiwa_model_.get_joint_frames()));
    this->torques_command_ = std::make_shared<StateRepresentation::JointTorques>(StateRepresentation::JointTorques::Zero("iiwa", iiwa_model_.get_joint_frames()));
    this->add_parameter(this->compliant_mode_);
    this->add_parameter(this->desired_radial_force_);
    this->add_parameters(this->controller_.get_parameters());
  }

  bool on_configure() {
    this->add_subscription<geometry_msgs::msg::TwistStamped>("/ds/desired_twist", this->desired_state_);
    this->add_subscription<geometry_msgs::msg::WrenchStamped>("/calibrated_ft_sensor/iiwa_link_ee_polish", this->feedback_state_);
    this->add_subscription<sensor_msgs::msg::JointState>("/iiwa/joint_states", this->current_robot_state_);
    this->add_publisher<std_msgs::msg::Float64MultiArray>("/iiwa/TorqueController/raw", this->torques_command_);
    return true;
  }

  void step() {
    // send a zero torque by default
    *this->torques_command_ = StateRepresentation::JointTorques::Zero("iiwa", iiwa_model_.get_joint_frames());
    // if we received the current state we proceed with additional computations
    if (!this->current_robot_state_->is_empty()) {
      // we can remove inertia and coriolis effect
      // *this->torques_command_ -= iiwa_model_.compute_inertia_torques(*this->current_robot_state_);
      // *this->torques_command_ -= iiwa_model_.compute_coriolis_torques(*this->current_robot_state_);
      // compute the command if we received the desired twist and the robot is not in compliant mode
      if (!this->desired_state_->is_empty() && !this->compliant_mode_->get_value()) {
        StateRepresentation::CartesianPose current_eef_pose = this->lookup_transform("iiwa_link_ee_polish",
                                                                                        "iiwa_link_0");
        StateRepresentation::CartesianPose polishing_wheel_pose = this->lookup_transform("polishing_wheel",
                                                                                        "iiwa_link_0");
        Eigen::Vector3d force_direction = polishing_wheel_pose.get_position() - current_eef_pose.get_position();
        this->desired_state_->set_force(this->desired_radial_force_->get_value() * force_direction);        
        // use the model forward kinematic to extract eef CartesianTwist
        StateRepresentation::Jacobian jacobian = this->iiwa_model_.compute_jacobian(*this->current_robot_state_);
        // update the current state
        this->feedback_state_->set_pose(current_eef_pose.get_pose());
        // compute current twist
        StateRepresentation::CartesianState current_twist = jacobian * static_cast<StateRepresentation::JointVelocities>(*this->current_robot_state_);
        this->feedback_state_->set_twist(current_twist.get_twist());
        //get the wrench command from the controller
        *this->torques_command_ += this->controller_.compute_command(*this->desired_state_, *this->feedback_state_, jacobian);
      }
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

  std::shared_ptr<FTSensorInterface> fts = std::make_shared<FTSensorInterface>("ft_sensor_interface", 1ms);
  std::shared_ptr<LinearMotionGenerator> lmg = std::make_shared<LinearMotionGenerator>("motion_generator", 5ms);
  std::shared_ptr<RobotInterface> sri = std::make_shared<RobotInterface>("robot_interface", 5ms);

  exe.add_node(fts->get_node_base_interface());
  exe.add_node(lmg->get_node_base_interface());
  exe.add_node(sri->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return 0;
}