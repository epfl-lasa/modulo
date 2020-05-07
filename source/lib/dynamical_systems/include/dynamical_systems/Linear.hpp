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
	private:
		std::shared_ptr<StateRepresentation::Parameter<S>> attractor_; ///< attractor of the dynamical system in the space
		std::shared_ptr<StateRepresentation::Parameter<double>> gain_; ///< gain associate to the system
	
	public:
		/**
		 * @brief Empty constructor
		 */
		explicit Linear(const S& attractor, double gain=1.0);

		/**
		 * @brief Getter of the attractor
		 * @return the attractor as a const reference
		 */
		const S& get_attractor() const;

		/**
		 * @brief Setter of the attractor as a new value
		 * @param attractor the new attractor
		 */
		void set_attractor(const S& attractor);

		/**
		 * @brief Getter of the gain attribute
		 * @return The gain value 
		 */
		double get_gain() const;

		/**
		 * @brief Getter of the gain attribute
		 * @return The gain value 
		 */
		void set_gain(double gain);

		/**
		 * @brief Evaluate the value of the dynamical system at a given state
		 * @param state state at wich to perform the evaluation
		 * @return the state (velocity) to move toward the attractor
		 */
		const S evaluate(const S& state) const override;

		/**
		 * @brief Return a list of all the parameters of the dynamical system
		 * @return the list of parameters
		 */
		const std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> get_parameters() const;
	};

	template<>
	Linear<StateRepresentation::CartesianState>::Linear(const StateRepresentation::CartesianState& attractor, double gain):
	DynamicalSystem<StateRepresentation::CartesianState>(),
	attractor_(std::make_shared<StateRepresentation::Parameter<StateRepresentation::CartesianState>>(StateRepresentation::Parameter<StateRepresentation::CartesianPose>("attractor", attractor))),
	gain_(std::make_shared<StateRepresentation::Parameter<double>>("gain", gain))
	{}

	template<>
	Linear<StateRepresentation::JointState>::Linear(const StateRepresentation::JointState& attractor, double gain):
	DynamicalSystem<StateRepresentation::JointState>(),
	attractor_(std::make_shared<StateRepresentation::Parameter<StateRepresentation::JointState>>(StateRepresentation::Parameter<StateRepresentation::JointState>("attractor", attractor))),
	gain_(std::make_shared<StateRepresentation::Parameter<double>>("gain", gain))
	{}

	template<class S>
	inline const S& Linear<S>::get_attractor() const
	{
		return this->attractor_->get_value();
	}

	template<class S>
	inline void Linear<S>::set_attractor(const S& attractor)
	{
		this->attractor_->set_value(attractor);
	}

	template<class S>
	inline double Linear<S>::get_gain() const
	{
		return this->gain_->get_value();
	}

	template<class S>
	inline void Linear<S>::set_gain(double gain)
	{
		this->gain_->set_value(gain);
	}

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

	template<class S>
	const std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> Linear<S>::get_parameters() const
	{
		std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> param_list;
		param_list.push_back(this->attractor_);
		param_list.push_back(this->gain_);
		return param_list;
	}
}
