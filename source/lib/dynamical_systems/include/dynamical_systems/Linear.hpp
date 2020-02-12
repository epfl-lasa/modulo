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
	/**
	 * @class Linear
	 * @brief Represent a Linear dynamical system to move toward an attractor
	 * @tparam S the type of space of the dynamical system (e.g. Cartesian or Joint)
	 */
	template<class S>
	class Linear: public DynamicalSystem<S>
	{
	private:
		std::shared_ptr<S> attractor_; //< attractor of the dynamical system in the space

	public:
		/**
		 * @brief Constructor with a provided gain 
		 * @param  gain the gain of the dynamical system (default = 1)
		 */
		explicit Linear(double gain=1);

		/**
		 * @brief Getter of the attractor
		 * @return the attractor as a const reference
		 */
		const S& get_attractor() const;

		/**
		 * @brief Setter of the attractor as a new shared_ptr
		 * @param attractor the new attractor
		 */
		void set_attractor(const std::shared_ptr<S>& attractor);

		/**
		 * @brief Setter of the attractor as a new value
		 * @param attractor the new attractor
		 */
		void set_attractor(const S& attractor);

		/**
		 * @brief Evaluate the value of the dynamical system at a given state
		 * @param state state at wich to perform the evaluation
		 * @return the state (velocity) to move toward the attractor
		 */
		const S evaluate(const S& state) const;

		/**
		 * @brief Evaluate the value of the dynamical system at a given state
		 * @param state state at wich to perform the evaluation (given as a shared_ptr)
		 * @return the state (velocity) to move toward the attractor
		 */
		const S evaluate(const std::shared_ptr<S>& state) const;
	};

	template<class S>
	Linear<S>::Linear(double gain):
	DynamicalSystem<S>(gain),
	attractor_(std::make_shared<S>())
	{}

	template<class S>
	inline const S& Linear<S>::get_attractor() const
	{
		return *this->attractor_;
	}

	template<class S>
	inline void Linear<S>::set_attractor(const std::shared_ptr<S>& attractor)
	{
		this->attractor_ = attractor;
	}

	template<class S>
	inline void Linear<S>::set_attractor(const S& attractor)
	{
		*this->attractor_ = attractor;
	}

	template<>
	const StateRepresentation::CartesianState Linear<StateRepresentation::CartesianState>::evaluate(const StateRepresentation::CartesianState& state) const
	{
		StateRepresentation::CartesianTwist twist = static_cast<const StateRepresentation::CartesianPose&>(state) - static_cast<const StateRepresentation::CartesianPose&>(this->get_attractor());
		twist *= -this->get_gain();
		return twist;
	}

	template<>
	const StateRepresentation::CartesianState Linear<StateRepresentation::CartesianState>::evaluate(const std::shared_ptr<StateRepresentation::CartesianState>& state) const
	{
		return this->evaluate(*state);
	}

	template<>
	const StateRepresentation::JointState Linear<StateRepresentation::JointState>::evaluate(const StateRepresentation::JointState& state) const
	{
		StateRepresentation::JointState positions = - this->get_gain() * (state - this->get_attractor());
		StateRepresentation::JointState velocities(state.get_name(), state.get_names());
		velocities.set_velocities(positions.get_positions());
		return velocities;
	}

	template<>
	const StateRepresentation::JointState Linear<StateRepresentation::JointState>::evaluate(const std::shared_ptr<StateRepresentation::JointState>& state) const
	{
		return this->evaluate(*state);
	}
}
#endif