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
 * @param new_state
 * @throw MessageTranslationException
 * @return
 */
template<typename T>
inline std::shared_ptr<T> dynamic_cast_with_check(
    std::shared_ptr<state_representation::State>& state, std::shared_ptr<state_representation::State>& new_state
) {
  auto derived_state = std::dynamic_pointer_cast<T>(state->shared_from_this());
  if (derived_state == nullptr) {
    throw modulo_core::exceptions::MessageTranslationException("Dynamic casting of state failed.");
  }
  *derived_state = *std::dynamic_pointer_cast<T>(new_state);
  return derived_state;
}

template<>
inline void read_message(std::shared_ptr<state_representation::State>& state, const EncodedState& message) {
  using namespace state_representation;
  std::string tmp(message.data.begin(), message.data.end());
  auto new_state = clproto::decode<std::shared_ptr<State>>(tmp);
  switch (new_state->get_type()) {
    case StateType::STATE:
      *state = *new_state;
      break;
    case StateType::SPATIAL_STATE:
      state = dynamic_cast_with_check<SpatialState>(state, new_state);
      break;
    case StateType::CARTESIAN_STATE:
      state = dynamic_cast_with_check<CartesianState>(state, new_state);
      break;
    case StateType::CARTESIAN_POSE:
      state =
          state->get_type() == StateType::CARTESIAN_STATE ? dynamic_cast_with_check<CartesianState>(state, new_state)
                                                          : dynamic_cast_with_check<CartesianPose>(state, new_state);
      break;
    case StateType::CARTESIAN_TWIST:
      state =
          state->get_type() == StateType::CARTESIAN_STATE ? dynamic_cast_with_check<CartesianState>(state, new_state)
                                                          : dynamic_cast_with_check<CartesianTwist>(state, new_state);
      break;
    case StateType::CARTESIAN_ACCELERATION:
      state =
          state->get_type() == StateType::CARTESIAN_STATE ? dynamic_cast_with_check<CartesianState>(state, new_state)
                                                          : dynamic_cast_with_check<CartesianAcceleration>(
              state, new_state
          );
      break;
    case StateType::CARTESIAN_WRENCH:
      state =
          state->get_type() == StateType::CARTESIAN_STATE ? dynamic_cast_with_check<CartesianState>(state, new_state)
                                                          : dynamic_cast_with_check<CartesianWrench>(state, new_state);
      break;
    case StateType::JACOBIAN:
      state = dynamic_cast_with_check<Jacobian>(state, new_state);
      break;
    case StateType::JOINT_STATE:
      state = dynamic_cast_with_check<JointState>(state, new_state);
      break;
    case StateType::JOINT_POSITIONS:
      state = state->get_type() == StateType::JOINT_STATE ? dynamic_cast_with_check<JointState>(state, new_state)
                                                          : dynamic_cast_with_check<JointPositions>(state, new_state);
      break;
    case StateType::JOINT_VELOCITIES:
      state = state->get_type() == StateType::JOINT_STATE ? dynamic_cast_with_check<JointState>(state, new_state)
                                                          : dynamic_cast_with_check<JointVelocities>(state, new_state);
      break;
    case StateType::JOINT_ACCELERATIONS:
      state = state->get_type() == StateType::JOINT_STATE ? dynamic_cast_with_check<JointState>(state, new_state)
                                                          : dynamic_cast_with_check<JointAccelerations>(
              state, new_state
          );
      break;
    case StateType::JOINT_TORQUES:
      state = state->get_type() == StateType::JOINT_STATE ? dynamic_cast_with_check<JointState>(state, new_state)
                                                          : dynamic_cast_with_check<JointTorques>(state, new_state);
      break;
    case StateType::PARAMETER: {
      auto param_ptr = std::dynamic_pointer_cast<ParameterInterface>(new_state);
      switch (param_ptr->get_parameter_type()) {
        case ParameterType::BOOL:
          state = dynamic_cast_with_check<Parameter<bool>>(state, new_state);
          break;
        case ParameterType::BOOL_ARRAY:
          state = dynamic_cast_with_check<Parameter<std::vector<bool>>>(state, new_state);
          break;
        case ParameterType::INT:
          state = dynamic_cast_with_check<Parameter<int>>(state, new_state);
          break;
        case ParameterType::INT_ARRAY:
          state = dynamic_cast_with_check<Parameter<std::vector<int>>>(state, new_state);
          break;
        case ParameterType::DOUBLE:
          state = dynamic_cast_with_check<Parameter<double>>(state, new_state);
          break;
        case ParameterType::DOUBLE_ARRAY:
          state = dynamic_cast_with_check<Parameter<std::vector<double>>>(state, new_state);
          break;
        case ParameterType::STRING:
          state = dynamic_cast_with_check<Parameter<std::string>>(state, new_state);
          break;
        case ParameterType::STRING_ARRAY:
          state = dynamic_cast_with_check<Parameter<std::vector<std::string>>>(state, new_state);
          break;
        case ParameterType::VECTOR:
          state = dynamic_cast_with_check<Parameter<Eigen::VectorXd>>(state, new_state);
          break;
        case ParameterType::MATRIX:
          state = dynamic_cast_with_check<Parameter<Eigen::MatrixXd>>(state, new_state);
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
