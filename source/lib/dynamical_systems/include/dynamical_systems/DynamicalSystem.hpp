/**
 * @author Baptiste Busch
 * @date 2019/07/18
 *
 */

#pragma once

#include <list>
#include <memory>
#include "state_representation/Parameters/ParameterInterface.hpp"


namespace DynamicalSystems
{
	/**
	 * @class DynamicalSystem
	 * @brief Abstract class to define a DynamicalSystem either in joint or cartesian spaces
	 */
	template<class S>
	class DynamicalSystem 
	{
	public:
		/**
		 * @brief Constructor with a provided gain
		 * @param attractor attractor of the dynamical system
		 */
		explicit DynamicalSystem();

		/**
		 * @brief Evaluate the value of the dynamical system at a given state
		 * @param state state at wich to perform the evaluation
		 * @return the state (velocity) to move toward the attractor
		 */
		virtual const S evaluate(const S& state) const=0;

		/**
		 * @brief Return a list of all the parameters of the dynamical system
		 * @return the list of parameters
		 */
		virtual const std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> get_parameters() const;

		/**
		 * @brief Get the reference frame in wich the dynamic is expressed,
		 * default world, not relevent for dynamic in joint state
		 */
		virtual const std::string get_reference_frame() const;
	};

	template<class S>
	DynamicalSystem<S>::DynamicalSystem()
	{}

	template<class S>
	const std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> DynamicalSystem<S>::get_parameters() const
	{
		std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> param_list;
		return param_list;
	}

	template<class S>
	inline const std::string DynamicalSystem<S>::get_reference_frame() const
	{
		return "world";
	}
}
