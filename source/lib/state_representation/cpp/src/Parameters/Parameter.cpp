#include "state_representation/Parameters/Parameter.hpp"
#include "state_representation/Space/Cartesian/CartesianPose.hpp"
#include "state_representation/Robot/JointPositions.hpp"

namespace StateRepresentation
{
	template <>
	Parameter<double>::Parameter(const std::string& name):
	State(StateType::PARAMETER_DOUBLE, name)
	{
		this->set_filled();
	}

	template <>
	Parameter<double>::Parameter(const std::string& name, const double& value):
	State(StateType::PARAMETER_DOUBLE, name), value(value)
	{
		this->set_filled();
	}

	template <>
	Parameter<std::vector<double>>::Parameter(const std::string& name):
	State(StateType::PARAMETER_DOUBLE_ARRAY, name)
	{}

	template <>
	Parameter<std::vector<double>>::Parameter(const std::string& name, const std::vector<double>& value):
	State(StateType::PARAMETER_DOUBLE_ARRAY, name), value(value)
	{
		this->set_filled();
	}

	template <>
	Parameter<bool>::Parameter(const std::string& name):
	State(StateType::PARAMETER_BOOL, name)
	{}

	template <>
	Parameter<bool>::Parameter(const std::string& name, const bool& value):
	State(StateType::PARAMETER_BOOL, name), value(value)
	{
		this->set_filled();
	}

	template <>
	Parameter<std::vector<bool>>::Parameter(const std::string& name):
	State(StateType::PARAMETER_BOOL_ARRAY, name)
	{}

	template <>
	Parameter<std::vector<bool>>::Parameter(const std::string& name, const std::vector<bool>& value):
	State(StateType::PARAMETER_BOOL_ARRAY, name), value(value)
	{
		this->set_filled();
	}

	template <>
	Parameter<std::string>::Parameter(const std::string& name):
	State(StateType::PARAMETER_STRING, name)
	{}

	template <>
	Parameter<std::string>::Parameter(const std::string& name, const std::string& value):
	State(StateType::PARAMETER_STRING, name), value(value)
	{
		this->set_filled();
	}

	template <>
	Parameter<std::vector<std::string>>::Parameter(const std::string& name):
	State(StateType::PARAMETER_STRING_ARRAY, name)
	{}

	template <>
	Parameter<std::vector<std::string>>::Parameter(const std::string& name, const std::vector<std::string>& value):
	State(StateType::PARAMETER_STRING_ARRAY, name), value(value)
	{
		this->set_filled();
	}

	template <>
	Parameter<CartesianPose>::Parameter(const std::string& name):
	State(StateType::PARAMETER_CARTESIANPOSE, name)
	{}

	template <>
	Parameter<CartesianPose>::Parameter(const std::string& name, const CartesianPose& value):
	State(StateType::PARAMETER_CARTESIANPOSE, name), value(value)
	{
		this->set_filled();
	}

	template <>
	Parameter<JointPositions>::Parameter(const std::string& name):
	State(StateType::PARAMETER_JOINTPOSITIONS, name)
	{}

	template <>
	Parameter<JointPositions>::Parameter(const std::string& name, const JointPositions& value):
	State(StateType::PARAMETER_JOINTPOSITIONS, name), value(value)
	{
		this->set_filled();
	}
}