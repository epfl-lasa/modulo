#pragma once

#include <rclcpp/rclcpp.hpp>
#include <state_representation/parameters/Parameter.hpp>

namespace modulo_new_core::translators {

/**
 * @brief Write a ROS Parameter from a ParameterInterface pointer.
 * @param parameter the ParameterInterface pointer with a name and value
 * @return A new ROS Parameter object
 */
rclcpp::Parameter write_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter);

/**
 * @brief Create a new ParameterInterface from a ROS Parameter object.
 * @param ros_parameter the ROS parameter object to read
 * @return A new ParameterInterface pointer
 */
std::shared_ptr<state_representation::ParameterInterface> read_parameter(const rclcpp::Parameter& ros_parameter);

/**
 * @brief Update the parameter value of a ParameterInterface from a ROS Parameter object.
 * @details The destination ParameterInterface must have a compatible parameter name and type.
 * @param ros_parameter the ROS parameter object to read
 * @param parameter An existing ParameterInterface pointer which will have
 */
void read_parameter(
    const rclcpp::Parameter& ros_parameter, std::shared_ptr<state_representation::ParameterInterface>& parameter
);

}