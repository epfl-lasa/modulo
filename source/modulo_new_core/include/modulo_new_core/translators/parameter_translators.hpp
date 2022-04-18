#pragma once

#include <rclcpp/rclcpp.hpp>
#include <state_representation/parameters/Parameter.hpp>

namespace modulo_new_core::translators {

/**
 * @brief Copy the value of one parameter interface into another.
 * @details When referencing a Parameter instance through a ParameterInterface pointer, it is necessary
 * to know the parameter type to get or set the parameter value. Sometimes it is desirable to update the
 * parameter value from another compatible ParameterInterface, while still preserving the reference
 * to the original instance. This helper function calls the getters and setters of the appropriate parameter type
 * to modify the value of the parameter instance while preserving the reference of the original pointer.
 * @param source_parameter The ParameterInterface with a value to copy
 * @param parameter The destination ParameterInterface to be updated
 */
void copy_parameter_value(
    const std::shared_ptr<state_representation::ParameterInterface>& source_parameter,
    const std::shared_ptr<state_representation::ParameterInterface>& parameter
);

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

std::shared_ptr<state_representation::ParameterInterface> read_parameter_const(
    const rclcpp::Parameter& ros_parameter,
    const std::shared_ptr<const state_representation::ParameterInterface>& parameter
);

/**
 * @brief Update the parameter value of a ParameterInterface from a ROS Parameter object.
 * @details The destination ParameterInterface must have a compatible parameter name and type.
 * @param ros_parameter the ROS parameter object to read
 * @param parameter An existing ParameterInterface pointer
 */
void read_parameter(
    const rclcpp::Parameter& ros_parameter, const std::shared_ptr<state_representation::ParameterInterface>& parameter
);

}