/**
 * @class Linear
 * @brief Class to implement a Linear DynamicalSystem
 * @author Baptiste Busch
 * @date 2019/07/18
 *
 */

#ifndef DYNAMICAL_SYSTEMS_LINEAR_H_
#define DYNAMICAL_SYSTEMS_LINEAR_H_


#include "dynamical_systems/DynamicalSystem.hpp"

namespace DynamicalSystems
{
	template<class S>
	class Linear: public DynamicalSystem<S>
	{
	private:
		S attractor_;

	public:
		explicit Linear(double gain=1);

		const S& get_attractor() const;

		void set_attractor(const S& attractor);

		const S evaluate(const S& state) const;
	};

	template<class S>
	Linear<S>::Linear(double gain):
	DynamicalSystem<S>(gain)
	{}

	template<class S>
	inline const S& Linear<S>::get_attractor() const
	{
		return this->attractor_;
	}

	template<class S>
	inline void Linear<S>::set_attractor(const S& attractor)
	{
		this->attractor_ = attractor;
	}

	template<>
	const StateRepresentation::CartesianState Linear<StateRepresentation::CartesianState>::evaluate(const StateRepresentation::CartesianState& state) const
	{
		StateRepresentation::CartesianVelocity velocity = - this->get_gain() * (static_cast<const StateRepresentation::CartesianPose&>(state) - static_cast<const StateRepresentation::CartesianPose&>(this->get_attractor()));
		return velocity;
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
#endif