/**
 * @author Baptiste Busch
 * @date 2019/07/18
 *
 */

#pragma once

#include "dynamical_systems/DynamicalSystem.hpp"
#include "state_representation/Space/Cartesian/CartesianState.hpp"
#include "state_representation/Space/Cartesian/CartesianPose.hpp"
#include "state_representation/Space/Cartesian/CartesianTwist.hpp"
#include "state_representation/Robot/JointState.hpp"

namespace DynamicalSystems
{
	/**
	 * @class Linear
	 * @brief Represent a Linear dynamical system to move toward an attractor
	 * @tparam S the type of space of the dynamical system (e.g. Cartesian or Joint)
	 */
	template<class S>
	class Linear: public DynamicalSystem<S>
	{
	public:
		/**
		 * @brief Empty constructor
		 */
		explicit Linear();

		/**
		 * @brief Evaluate the value of the dynamical system at a given state
		 * @param state state at wich to perform the evaluation
		 * @return the state (velocity) to move toward the attractor
		 */
		const S evaluate(const S& state) const override;
	};

	template<>
	Linear<StateRepresentation::CartesianState>::Linear():
	DynamicalSystem<StateRepresentation::CartesianState>(std::make_shared<StateRepresentation::Parameter<StateRepresentation::CartesianState>>(StateRepresentation::Parameter<StateRepresentation::CartesianPose>("target")))
	{}

	template<>
	Linear<StateRepresentation::JointState>::Linear():
	DynamicalSystem<StateRepresentation::JointState>(std::make_shared<StateRepresentation::Parameter<StateRepresentation::JointState>>(StateRepresentation::Parameter<StateRepresentation::JointState>("target")))
	{}

	template<>
	const StateRepresentation::CartesianState Linear<StateRepresentation::CartesianState>::evaluate(const StateRepresentation::CartesianState& state) const
	{
		StateRepresentation::CartesianTwist twist = static_cast<const StateRepresentation::CartesianPose&>(state) - static_cast<const StateRepresentation::CartesianPose&>(this->get_attractor());
		twist *= -this->get_gain();
		return twist;
	}

	template<>
	const StateRepresentation::JointState Linear<StateRepresentation::JointState>::evaluate(const StateRepresentation::JointState& state) const
	{
		StateRepresentation::JointState positions = - this->get_gain() * (state - this->get_attractor());
		StateRepresentation::JointState velocities(state.get_name(), state.get_names());
		velocities.set_velocities(positions.get_positions());
		return velocities;
	}
}
