/**
 * @author Baptiste Busch
 * @date 2019/07/18
 *
 */

#pragma once

#include "dynamical_systems/DynamicalSystem.hpp"
#include "state_representation/Parameters/Parameter.hpp"
#include "state_representation/Space/Cartesian/CartesianState.hpp"
#include "state_representation/Space/Cartesian/CartesianPose.hpp"
#include "state_representation/Space/Cartesian/CartesianTwist.hpp"
#include "state_representation/Robot/JointState.hpp"
#include "state_representation/Robot/JointPositions.hpp"
#include "dynamical_systems/Exceptions/IncompatibleSizeException.hpp"

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
		std::shared_ptr<StateRepresentation::Parameter<Eigen::MatrixXd>> gain_; ///< gain associate to the system
	
	protected:
		/**
		 * @brief Compute the dynamics of the input state.
		 * Internal function, to be redefined based on the 
		 * type of dynamical system, called by the evaluate
		 * function
		 * @param state the input state
		 * @return the output state
		 */
		const S compute_dynamics(const S& state) const;

	public:
		/**
		 * @brief Constructor with specified attractor and iso gain
		 * @param attractor the attractor of the linear system
		 * @param iso_gain the iso gain of the system
		 */
		explicit Linear(const S& attractor, double iso_gain=1.0);

		/**
		 * @brief Constructor with specified attractor and different gains specified as diagonal coefficients
		 * @param attractor the attractor of the linear system
		 * @param diagonal_coefficients the gains values specified as diagonal coefficients
		 */
		explicit Linear(const S& attractor, const std::vector<double>& diagonal_coefficients);

		/**
		 * @brief Constructor with specified attractor and different gains specified as full gain matrix
		 * @param attractor the attractor of the linear system
		 * @param gain_matrix the gains values specified as a full matrix
		 */
		explicit Linear(const S& attractor, const Eigen::MatrixXd& gain_matrix);

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
		const Eigen::MatrixXd& get_gain() const;

		/**
		 * @brief Setter of the gain attribute
		 * @param iso_gain the gain value as an iso coefficient
		 */
		void set_gain(double iso_gain);

		/**
		 * @brief Setter of the gain attribute
		 * @param diagonal_coefficients the gain values as diagonal coefficients
		 */
		void set_gain(const std::vector<double>& diagonal_coefficients);

		/**
		 * @brief Setter of the gain attribute
		 * @param gain_matrix the gain values as a full gain matrix
		 */
		void set_gain(const Eigen::MatrixXd& gain_matrix);

		/**
		 * @brief Return a list of all the parameters of the dynamical system
		 * @return the list of parameters
		 */
		const std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> get_parameters() const;
	};

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
	inline const Eigen::MatrixXd& Linear<S>::get_gain() const
	{
		return this->gain_->get_value();
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
