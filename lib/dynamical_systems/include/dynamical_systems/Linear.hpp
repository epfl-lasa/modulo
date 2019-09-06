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
	inline void Linear<StateRepresentation::CartesianState>::set_attractor(const StateRepresentation::CartesianState& attractor)
	{
		this->attractor_ = attractor;
		for (unsigned int i=0; i<3; ++i) this->message_.set_parameter_values(i, this->attractor_.get_position()(i));
	}

	template<>
	const StateRepresentation::CartesianState Linear<StateRepresentation::CartesianState>::evaluate(const StateRepresentation::CartesianState& state) const
	{
		StateRepresentation::CartesianPose pose = - this->get_gain() * (static_cast<const StateRepresentation::CartesianPose&>(state) - static_cast<const StateRepresentation::CartesianPose&>(this->get_attractor()));
		StateRepresentation::CartesianVelocity velocity(state.get_name(), state.get_reference_frame());
		velocity.set_linear_velocity(pose.get_position());
		velocity.set_angular_velocity(pose.get_orientation().vec());
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