/**
 * @author Baptiste Busch
 * @date 2019/07/18
 *
 */

#pragma once

#include "dynamical_systems/Exceptions/NotImplementedException.hpp"
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
		std::shared_ptr<S> attractor_; ///< attractor of the dynamical system in the space
		double gain_; ///< gain associate to the system

	public:
		/**
		 * @brief Constructor with a provided gain 
		 * @param  gain the gain of the dynamical system (default = 1)
		 */
		explicit DynamicalSystem(double gain=1);

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
	};

	template<class S>
	DynamicalSystem<S>::DynamicalSystem(double gain):
	gain_(gain),
	attractor_(std::make_shared<S>())
	{}

	template<class S>
	inline const S& DynamicalSystem<S>::get_attractor() const
	{
		return *this->attractor_;
	}

	template<class S>
	inline void DynamicalSystem<S>::set_attractor(const std::shared_ptr<S>& attractor)
	{
		this->attractor_ = attractor;
	}

	template<class S>
	inline void DynamicalSystem<S>::set_attractor(const S& attractor)
	{
		*this->attractor_ = attractor;
	}


	template<class S>
	inline double DynamicalSystem<S>::get_gain() const
	{
		return this->gain_;
	}

	template<class S>
	inline void DynamicalSystem<S>::set_gain(double gain)
	{
		this->gain_ = gain;
	}

	template<class S>
	const S DynamicalSystem<S>::evaluate(const S& state) const
	{
		throw Exceptions::NotImplementedException("This method is not implemented for abstract base class");
		return state;
	}
}
