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
 */
template<typename T>
inline void read_message(T& state, const EncodedState& message) {
  std::string tmp(message.data.begin(), message.data.end());
  state = clproto::decode<T>(tmp);
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
    case StateType::SPATIAL_STATE: {
      auto derived_state = std::dynamic_pointer_cast<SpatialState>(state->shared_from_this());
      *derived_state = *std::dynamic_pointer_cast<SpatialState>(new_state);
      state = derived_state;
      break;
    }
    case StateType::CARTESIAN_STATE: {
      auto derived_state = std::dynamic_pointer_cast<CartesianState>(state->shared_from_this());
      *derived_state = *std::dynamic_pointer_cast<CartesianState>(new_state);
      state = derived_state;
      break;
    }
    case StateType::CARTESIAN_POSE: {
      auto derived_state = std::dynamic_pointer_cast<CartesianPose>(state->shared_from_this());
      *derived_state = *std::dynamic_pointer_cast<CartesianPose>(new_state);
      state = derived_state;
      break;
    }
    case StateType::CARTESIAN_TWIST: {
      auto derived_state = std::dynamic_pointer_cast<CartesianTwist>(state->shared_from_this());
      *derived_state = *std::dynamic_pointer_cast<CartesianTwist>(new_state);
      state = derived_state;
      break;
    }
    case StateType::CARTESIAN_ACCELERATION: {
      auto derived_state = std::dynamic_pointer_cast<CartesianAcceleration>(state->shared_from_this());
      *derived_state = *std::dynamic_pointer_cast<CartesianAcceleration>(new_state);
      state = derived_state;
      break;
    }
    case StateType::CARTESIAN_WRENCH: {
      auto derived_state = std::dynamic_pointer_cast<CartesianWrench>(state->shared_from_this());
      *derived_state = *std::dynamic_pointer_cast<CartesianWrench>(new_state);
      state = derived_state;
      break;
    }
    case StateType::JACOBIAN: {
      auto derived_state = std::dynamic_pointer_cast<Jacobian>(state->shared_from_this());
      *derived_state = *std::dynamic_pointer_cast<Jacobian>(new_state);
      state = derived_state;
      break;
    }
    case StateType::JOINT_STATE: {
      auto derived_state = std::dynamic_pointer_cast<JointState>(state->shared_from_this());
      *derived_state = *std::dynamic_pointer_cast<JointState>(new_state);
      state = derived_state;
      break;
    }
    case StateType::JOINT_POSITIONS: {
      auto derived_state = std::dynamic_pointer_cast<JointPositions>(state->shared_from_this());
      *derived_state = *std::dynamic_pointer_cast<JointPositions>(new_state);
      state = derived_state;
      break;
    }
    case StateType::JOINT_VELOCITIES: {
      auto derived_state = std::dynamic_pointer_cast<JointVelocities>(state->shared_from_this());
      *derived_state = *std::dynamic_pointer_cast<JointVelocities>(new_state);
      state = derived_state;
      break;
    }
    case StateType::JOINT_ACCELERATIONS: {
      auto derived_state = std::dynamic_pointer_cast<JointAccelerations>(state->shared_from_this());
      *derived_state = *std::dynamic_pointer_cast<JointAccelerations>(new_state);
      state = derived_state;
      break;
    }
    case StateType::JOINT_TORQUES: {
      auto derived_state = std::dynamic_pointer_cast<JointTorques>(state->shared_from_this());
      *derived_state = *std::dynamic_pointer_cast<JointTorques>(new_state);
      state = derived_state;
      break;
    }
    case StateType::PARAMETER: {
      auto param_ptr = std::dynamic_pointer_cast<ParameterInterface>(new_state);
      switch (param_ptr->get_parameter_type()) {
        case ParameterType::BOOL: {
          auto derived_state = std::dynamic_pointer_cast<Parameter<bool>>(state->shared_from_this());
          *derived_state = *std::dynamic_pointer_cast<Parameter<bool>>(new_state);
          state = derived_state;
          break;
        }
        case ParameterType::BOOL_ARRAY: {
          auto derived_state = std::dynamic_pointer_cast<Parameter<std::vector<bool>>>(state->shared_from_this());
          *derived_state = *std::dynamic_pointer_cast<Parameter<std::vector<bool>>>(new_state);
          state = derived_state;
          break;
        }
        case ParameterType::INT: {
          auto derived_state = std::dynamic_pointer_cast<Parameter<int>>(state->shared_from_this());
          *derived_state = *std::dynamic_pointer_cast<Parameter<int>>(new_state);
          state = derived_state;
          break;
        }
        case ParameterType::INT_ARRAY: {
          auto derived_state = std::dynamic_pointer_cast<Parameter<std::vector<int>>>(state->shared_from_this());
          *derived_state = *std::dynamic_pointer_cast<Parameter<std::vector<int>>>(new_state);
          state = derived_state;
          break;
        }
        case ParameterType::DOUBLE: {
          auto derived_state = std::dynamic_pointer_cast<Parameter<double>>(state->shared_from_this());
          *derived_state = *std::dynamic_pointer_cast<Parameter<double>>(new_state);
          state = derived_state;
          break;
        }
        case ParameterType::DOUBLE_ARRAY: {
          auto derived_state = std::dynamic_pointer_cast<Parameter<std::vector<double>>>(state->shared_from_this());
          *derived_state = *std::dynamic_pointer_cast<Parameter<std::vector<double>>>(new_state);
          state = derived_state;
          break;
        }
        case ParameterType::STRING: {
          auto derived_state = std::dynamic_pointer_cast<Parameter<std::string>>(state->shared_from_this());
          *derived_state = *std::dynamic_pointer_cast<Parameter<std::string>>(new_state);
          state = derived_state;
          break;
        }
        case ParameterType::STRING_ARRAY: {
          auto
              derived_state = std::dynamic_pointer_cast<Parameter<std::vector<std::string>>>(state->shared_from_this());
          *derived_state = *std::dynamic_pointer_cast<Parameter<std::vector<std::string>>>(new_state);
          state = derived_state;
          break;
        }
        case ParameterType::VECTOR: {
          auto derived_state = std::dynamic_pointer_cast<Parameter<Eigen::VectorXd>>(state->shared_from_this());
          *derived_state = *std::dynamic_pointer_cast<Parameter<Eigen::VectorXd>>(new_state);
          state = derived_state;
          break;
        }
        case ParameterType::MATRIX: {
          auto derived_state = std::dynamic_pointer_cast<Parameter<Eigen::MatrixXd>>(state->shared_from_this());
          *derived_state = *std::dynamic_pointer_cast<Parameter<Eigen::MatrixXd>>(new_state);
          state = derived_state;
          break;
        }
        default:
          throw std::invalid_argument(
              "The ParameterType contained by parameter " + param_ptr->get_name() + " is unsupported."
          );
      }
      break;
    }
    default:
      throw std::invalid_argument("The StateType contained by state " + new_state->get_name() + " is unsupported.");
  }
}
}// namespace modulo_core::translators
