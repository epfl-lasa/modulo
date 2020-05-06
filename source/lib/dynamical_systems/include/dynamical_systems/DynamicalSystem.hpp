/**
 * @author Baptiste Busch
 * @date 2019/07/18
 *
 */

#pragma once

#include "dynamical_systems/Exceptions/NotImplementedException.hpp"
#include "state_representation/Parameters/ParameterInterface.hpp"
#include <memory>

namespace DynamicalSystems
{
	/**
	 * @class DynamicalSystem
	 * @brief Abstract class to define a DynamicalSystem either in joint or cartesian spaces
	 */
	template<class S>
	class DynamicalSystem 
	{
	private:
		std::shared_ptr<StateRepresentation::Parameter<S>> attractor_; ///< attractor of the dynamical system in the space
		std::shared_ptr<StateRepresentation::Parameter<double>> gain_; ///< gain associate to the system

	public:
		/**
		 * @brief Constructor with a provided gain
		 * @param attractor attractor of the dynamical system
		 */
		explicit DynamicalSystem(const std::shared_ptr<StateRepresentation::Parameter<S>>& attractor);

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
		virtual const S evaluate(const S& state) const;

		/**
		 * @brief Return a list of all the parameters of the dynamical system
		 * @return the list of parameters
		 */
		virtual const std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> get_parameters() const;
	};

	template<class S>
	DynamicalSystem<S>::DynamicalSystem(const std::shared_ptr<StateRepresentation::Parameter<S>>& attractor):
	attractor_(attractor),
	gain_(std::make_shared<StateRepresentation::Parameter<double>>("gain", 1.))
	{}

	template<class S>
	inline const S& DynamicalSystem<S>::get_attractor() const
	{
		return this->attractor_->get_value();
	}

	template<class S>
	inline void DynamicalSystem<S>::set_attractor(const S& attractor)
	{
		this->attractor_->set_value(attractor);
	}

	template<class S>
	inline double DynamicalSystem<S>::get_gain() const
	{
		return this->gain_->get_value();
	}

	template<class S>
	inline void DynamicalSystem<S>::set_gain(double gain)
	{
		this->gain_->set_value(gain);
	}

	template<class S>
	const S DynamicalSystem<S>::evaluate(const S& state) const
	{
		throw Exceptions::NotImplementedException("This method is not implemented for abstract base class");
		return state;
	}

	template<class S>
	const std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> DynamicalSystem<S>::get_parameters() const
	{
		std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> param_list;
		param_list.push_back(this->attractor_);
		param_list.push_back(this->gain_);
		return param_list;
	}
}
