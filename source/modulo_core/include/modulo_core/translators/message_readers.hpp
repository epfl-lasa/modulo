#pragma once

#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <clproto.h>
#include <state_representation/parameters/Parameter.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/joint/JointPositions.hpp>
#include <state_representation/space/Jacobian.hpp>

#include "modulo_core/EncodedState.hpp"
#include "modulo_core/exceptions/MessageTranslationException.hpp"

namespace modulo_core::translators {

/**
 * @brief Convert a ROS geometry_msgs::msg::Accel to a CartesianState
 * @param state The CartesianState to populate
 * @param message The ROS message to read from
 */
void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::Accel& message);

/**
 * @brief Convert a ROS geometry_msgs::msg::AccelStamped to a CartesianState
 * @param state The CartesianState to populate
 * @param message The ROS message to read from
 */
void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::AccelStamped& message);

/**
 * @brief Convert a ROS geometry_msgs::msg::Pose to a CartesianState
 * @param state The CartesianState to populate
 * @param message The ROS message to read from
 */
void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::Pose& message);

/**
 * @brief Convert a ROS geometry_msgs::msg::PoseStamped to a CartesianState
 * @param state The CartesianState to populate
 * @param message The ROS message to read from
 */
void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::PoseStamped& message);

/**
 * @brief Convert a ROS geometry_msgs::msg::Transform to a CartesianState
 * @param state The CartesianState to populate
 * @param message The ROS message to read from
 */
void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::Transform& message);

/**
 * @brief Convert a ROS geometry_msgs::msg::TransformStamped to a CartesianState
 * @param state The CartesianState to populate
 * @param message The ROS message to read from
 */
void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::TransformStamped& message);

/**
 * @brief Convert a ROS geometry_msgs::msg::Twist to a CartesianState
 * @param state The CartesianState to populate
 * @param message The ROS message to read from
 */
void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::Twist& message);

/**
 * @brief Convert a ROS geometry_msgs::msg::TwistStamped to a CartesianState
 * @param state The CartesianState to populate
 * @param message The ROS message to read from
 */
void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::TwistStamped& message);

/**
 * @brief Convert a ROS geometry_msgs::msg::Wrench to a CartesianState
 * @param state The CartesianState to populate
 * @param message The ROS message to read from
 */
void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::Wrench& message);

/**
 * @brief Convert a ROS geometry_msgs::msg::WrenchStamped to a CartesianState
 * @param state The CartesianState to populate
 * @param message The ROS message to read from
 */
void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::WrenchStamped& message);

/**
 * @brief Convert a ROS sensor_msgs::msg::JointState to a JointState
 * @param state The JointState to populate
 * @param message The ROS message to read from
 */
void read_message(state_representation::JointState& state, const sensor_msgs::msg::JointState& message);

/**
 * @brief Template function to convert a ROS std_msgs::msg::T to a Parameter<T>
 * @tparam T All types of parameters supported in ROS std messages
 * @tparam U All types of parameters supported in ROS std messages
 * @param state The Parameter<T> to populate
 * @param message The ROS message to read from
 */
template<typename T, typename U>
void read_message(state_representation::Parameter<T>& state, const U& message) {
  state.set_value(message.data);
}

/**
 * @brief Convert a ROS std_msgs::msg::Bool to a boolean
 * @param state The state to populate
 * @param message The ROS message to read from
 */
void read_message(bool& state, const std_msgs::msg::Bool& message);

/**
 * @brief Convert a ROS std_msgs::msg::Float64 to a double
 * @param state The state to populate
 * @param message The ROS message to read from
 */
void read_message(double& state, const std_msgs::msg::Float64& message);

/**
 * @brief Convert a ROS std_msgs::msg::Float64MultiArray to a vector of double
 * @param state The state to populate
 * @param message The ROS message to read from
 */
void read_message(std::vector<double>& state, const std_msgs::msg::Float64MultiArray& message);

/**
 * @brief Convert a ROS std_msgs::msg::Int32 to an integer
 * @param state The state to populate
 * @param message The ROS message to read from
 */
void read_message(int& state, const std_msgs::msg::Int32& message);

/**
 * @brief Convert a ROS std_msgs::msg::String to a string
 * @param state The state to populate
 * @param message The ROS message to read from
 */
void read_message(std::string& state, const std_msgs::msg::String& message);

/**
 * @brief Convert a ROS std_msgs::msg::UInt8MultiArray message to a State using protobuf decoding
 * @tparam T A state_representation::State type
 * @param state The state to populate
 * @param message The ROS message to read from
 * @throws MessageTranslationException if the translation failed or is not supported.
 */
template<typename T>
inline void read_message(T& state, const EncodedState& message) {
  try {
    std::string tmp(message.data.begin(), message.data.end());
    state = clproto::decode<T>(tmp);
  } catch (const std::exception& ex) {
    throw exceptions::MessageTranslationException(ex.what());
  }
}

/**
 * @brief
 * @tparam T
 * @param state
 * @return
 */
template<typename T>
inline std::shared_ptr<T> safe_dynamic_pointer_cast(std::shared_ptr<state_representation::State>& state) {
  auto derived_state_ptr = std::dynamic_pointer_cast<T>(state);
  if (derived_state_ptr == nullptr) {
    throw modulo_core::exceptions::MessageTranslationException("Dynamic casting of state failed.");
  }
  return derived_state_ptr;
}

/**
 * @brief
 * @tparam T
 * @param state
 * @param new_state
 */
template<typename T>
inline void safe_dynamic_cast(
    std::shared_ptr<state_representation::State>& state, std::shared_ptr<state_representation::State>& new_state
) {
  auto derived_state_ptr = safe_dynamic_pointer_cast<T>(state);
  auto received_state_ptr = safe_dynamic_pointer_cast<T>(new_state);
  *derived_state_ptr = *received_state_ptr;
}

template<>
inline void read_message(std::shared_ptr<state_representation::State>& state, const EncodedState& message) {
  using namespace state_representation;
  std::string tmp(message.data.begin(), message.data.end());
  auto new_state = clproto::decode<std::shared_ptr<State>>(tmp);
  switch (state->get_type()) {
    case StateType::STATE:
      *state = *new_state;
      break;
    case StateType::SPATIAL_STATE:
      safe_dynamic_cast<SpatialState>(state, new_state);
      break;
    case StateType::CARTESIAN_STATE: {
      auto derived_state = safe_dynamic_pointer_cast<CartesianState>(state);
      if (new_state->get_type() == StateType::CARTESIAN_POSE) {
        auto derived_new_state = safe_dynamic_pointer_cast<CartesianPose>(new_state);
        auto tmp_new_state = CartesianState(derived_new_state->get_name(), derived_new_state->get_reference_frame());
        tmp_new_state.set_pose(derived_new_state->get_pose());
        *derived_state = tmp_new_state;
      } else if (new_state->get_type() == StateType::CARTESIAN_TWIST) {
        auto derived_new_state = safe_dynamic_pointer_cast<CartesianTwist>(new_state);
        auto tmp_new_state = CartesianState(derived_new_state->get_name(), derived_new_state->get_reference_frame());
        tmp_new_state.set_twist(derived_new_state->get_twist());
        *derived_state = tmp_new_state;
      } else if (new_state->get_type() == StateType::CARTESIAN_ACCELERATION) {
        auto derived_new_state = safe_dynamic_pointer_cast<CartesianAcceleration>(new_state);
        auto tmp_new_state = CartesianState(derived_new_state->get_name(), derived_new_state->get_reference_frame());
        tmp_new_state.set_acceleration(derived_new_state->get_acceleration());
        *derived_state = tmp_new_state;
      } else if (new_state->get_type() == StateType::CARTESIAN_WRENCH) {
        auto derived_new_state = safe_dynamic_pointer_cast<CartesianWrench>(new_state);
        auto tmp_new_state = CartesianState(derived_new_state->get_name(), derived_new_state->get_reference_frame());
        tmp_new_state.set_wrench(derived_new_state->get_wrench());
        *derived_state = tmp_new_state;
      } else {
        safe_dynamic_cast<CartesianState>(state, new_state);
      }
      break;
    }
    case StateType::CARTESIAN_POSE: {
      if (new_state->get_type() == StateType::CARTESIAN_STATE) {
        auto derived_state = safe_dynamic_pointer_cast<CartesianPose>(state);
        auto derived_new_state = safe_dynamic_pointer_cast<CartesianState>(new_state);
        auto tmp_new_state = CartesianPose(
            derived_new_state->get_name(), derived_new_state->get_position(), derived_new_state->get_orientation(),
            derived_new_state->get_reference_frame());
        *derived_state = tmp_new_state;
      } else {
        safe_dynamic_cast<CartesianPose>(state, new_state);
      }
      break;
    }
    case StateType::CARTESIAN_TWIST: {
      if (new_state->get_type() == StateType::CARTESIAN_STATE) {
        auto derived_state = safe_dynamic_pointer_cast<CartesianTwist>(state);
        auto derived_new_state = safe_dynamic_pointer_cast<CartesianState>(new_state);
        auto tmp_new_state = CartesianTwist(
            derived_new_state->get_name(), derived_new_state->get_twist(), derived_new_state->get_reference_frame());
        *derived_state = tmp_new_state;
      } else {
        safe_dynamic_cast<CartesianTwist>(state, new_state);
      }
      break;
    }
    case StateType::CARTESIAN_ACCELERATION: {
      if (new_state->get_type() == StateType::CARTESIAN_STATE) {
        auto derived_state = safe_dynamic_pointer_cast<CartesianAcceleration>(state);
        auto derived_new_state = safe_dynamic_pointer_cast<CartesianState>(new_state);
        auto tmp_new_state = CartesianAcceleration(
            derived_new_state->get_name(), derived_new_state->get_acceleration(),
            derived_new_state->get_reference_frame());
        *derived_state = tmp_new_state;
      } else {
        safe_dynamic_cast<CartesianAcceleration>(state, new_state);
      }
      break;
    }
    case StateType::CARTESIAN_WRENCH: {
      if (new_state->get_type() == StateType::CARTESIAN_STATE) {
        auto derived_state = safe_dynamic_pointer_cast<CartesianWrench>(state);
        auto derived_new_state = safe_dynamic_pointer_cast<CartesianState>(new_state);
        auto tmp_new_state = CartesianWrench(
            derived_new_state->get_name(), derived_new_state->get_wrench(), derived_new_state->get_reference_frame());
        *derived_state = tmp_new_state;
      } else {
        safe_dynamic_cast<CartesianWrench>(state, new_state);
      }
      break;
    }
    case StateType::JACOBIAN:
      safe_dynamic_cast<Jacobian>(state, new_state);
      break;
    case StateType::JOINT_STATE: {
      auto derived_state = safe_dynamic_pointer_cast<JointState>(state);
      if (new_state->get_type() == StateType::JOINT_POSITIONS) {
        auto derived_new_state = safe_dynamic_pointer_cast<JointPositions>(new_state);
        auto tmp_new_state = JointState(derived_new_state->get_name(), derived_new_state->get_names());
        tmp_new_state.set_positions(derived_new_state->get_positions());
        *derived_state = tmp_new_state;
      } else if (new_state->get_type() == StateType::JOINT_VELOCITIES) {
        auto derived_new_state = safe_dynamic_pointer_cast<JointVelocities>(new_state);
        auto tmp_new_state = JointState(derived_new_state->get_name(), derived_new_state->get_names());
        tmp_new_state.set_velocities(derived_new_state->get_velocities());
        *derived_state = tmp_new_state;
      } else if (new_state->get_type() == StateType::JOINT_ACCELERATIONS) {
        auto derived_new_state = safe_dynamic_pointer_cast<JointAccelerations>(new_state);
        auto tmp_new_state = JointState(derived_new_state->get_name(), derived_new_state->get_names());
        tmp_new_state.set_accelerations(derived_new_state->get_accelerations());
        *derived_state = tmp_new_state;
      } else if (new_state->get_type() == StateType::JOINT_TORQUES) {
        auto derived_new_state = safe_dynamic_pointer_cast<JointTorques>(new_state);
        auto tmp_new_state = JointState(derived_new_state->get_name(), derived_new_state->get_names());
        tmp_new_state.set_torques(derived_new_state->get_torques());
        *derived_state = tmp_new_state;
      } else {
        safe_dynamic_cast<JointState>(state, new_state);
      }
      break;
    }
    case StateType::JOINT_POSITIONS: {
      if (new_state->get_type() == StateType::JOINT_STATE) {
        auto derived_state = safe_dynamic_pointer_cast<JointPositions>(state);
        auto derived_new_state = safe_dynamic_pointer_cast<JointState>(new_state);
        auto tmp_new_state = JointPositions(
            derived_new_state->get_name(), derived_new_state->get_names(), derived_new_state->get_positions());
        *derived_state = tmp_new_state;
      } else {
        safe_dynamic_cast<JointPositions>(state, new_state);
      }
      break;
    }
    case StateType::JOINT_VELOCITIES: {
      if (new_state->get_type() == StateType::JOINT_STATE) {
        auto derived_state = safe_dynamic_pointer_cast<JointVelocities>(state);
        auto derived_new_state = safe_dynamic_pointer_cast<JointState>(new_state);
        auto tmp_new_state = JointVelocities(
            derived_new_state->get_name(), derived_new_state->get_names(), derived_new_state->get_velocities());
        *derived_state = tmp_new_state;
      } else {
        safe_dynamic_cast<JointVelocities>(state, new_state);
      }
      break;
    }
    case StateType::JOINT_ACCELERATIONS: {
      if (new_state->get_type() == StateType::JOINT_STATE) {
        auto derived_state = safe_dynamic_pointer_cast<JointAccelerations>(state);
        auto derived_new_state = safe_dynamic_pointer_cast<JointState>(new_state);
        auto tmp_new_state = JointAccelerations(
            derived_new_state->get_name(), derived_new_state->get_names(), derived_new_state->get_accelerations());
        *derived_state = tmp_new_state;
      } else {
        safe_dynamic_cast<JointAccelerations>(state, new_state);
      }
      break;
    }
    case StateType::JOINT_TORQUES: {
      if (new_state->get_type() == StateType::JOINT_STATE) {
        auto derived_state = safe_dynamic_pointer_cast<JointTorques>(state);
        auto derived_new_state = safe_dynamic_pointer_cast<JointState>(new_state);
        auto tmp_new_state = JointTorques(
            derived_new_state->get_name(), derived_new_state->get_names(), derived_new_state->get_torques());
        *derived_state = tmp_new_state;
      } else {
        safe_dynamic_cast<JointTorques>(state, new_state);
      }
      break;
    }
    case StateType::PARAMETER: {
      auto param_ptr = std::dynamic_pointer_cast<ParameterInterface>(new_state);
      switch (param_ptr->get_parameter_type()) {
        case ParameterType::BOOL:
          safe_dynamic_cast<Parameter<bool>>(state, new_state);
          break;
        case ParameterType::BOOL_ARRAY:
          safe_dynamic_cast<Parameter<std::vector<bool>>>(state, new_state);
          break;
        case ParameterType::INT:
          safe_dynamic_cast<Parameter<int>>(state, new_state);
          break;
        case ParameterType::INT_ARRAY:
          safe_dynamic_cast<Parameter<std::vector<int>>>(state, new_state);
          break;
        case ParameterType::DOUBLE:
          safe_dynamic_cast<Parameter<double>>(state, new_state);
          break;
        case ParameterType::DOUBLE_ARRAY:
          safe_dynamic_cast<Parameter<std::vector<double>>>(state, new_state);
          break;
        case ParameterType::STRING:
          safe_dynamic_cast<Parameter<std::string>>(state, new_state);
          break;
        case ParameterType::STRING_ARRAY:
          safe_dynamic_cast<Parameter<std::vector<std::string>>>(state, new_state);
          break;
        case ParameterType::VECTOR:
          safe_dynamic_cast<Parameter<Eigen::VectorXd>>(state, new_state);
          break;
        case ParameterType::MATRIX:
          safe_dynamic_cast<Parameter<Eigen::MatrixXd>>(state, new_state);
          break;
        default:
          throw exceptions::MessageTranslationException(
              "The ParameterType contained by parameter " + param_ptr->get_name() + " is unsupported."
          );
      }
      break;
    }
    default:
      throw exceptions::MessageTranslationException(
          "The StateType contained by state " + new_state->get_name() + " is unsupported."
      );
  }
}
}// namespace modulo_core::translators
