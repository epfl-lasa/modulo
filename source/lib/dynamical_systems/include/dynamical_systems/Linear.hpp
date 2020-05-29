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
	inline void Linear<StateRepresentation::CartesianState>::set_gain(double iso_gain)
	{
		this->gain_->set_value(iso_gain * Eigen::MatrixXd::Identity(6, 6));
	}


	template<>
	inline void Linear<StateRepresentation::JointState>::set_gain(double iso_gain)
	{
		int nb_joints = this->get_attractor().get_size();
		this->gain_->set_value(iso_gain * Eigen::MatrixXd::Identity(nb_joints, nb_joints));
	}

	template<>
	inline void Linear<StateRepresentation::CartesianState>::set_gain(const std::vector<double>& diagonal_coefficients)
	{
		if(diagonal_coefficients.size() != 6)
		{
			throw Exceptions::IncompatibleSizeException("The provided diagonal coefficients do not correspond to the expected size of 6 elements");
		}
		Eigen::VectorXd diagonal = Eigen::VectorXd::Map(diagonal_coefficients.data(), 6);
		this->gain_->set_value(diagonal.asDiagonal());
	}

	template<>
	inline void Linear<StateRepresentation::JointState>::set_gain(const std::vector<double>& diagonal_coefficients)
	{
		size_t nb_joints = this->get_attractor().get_size();
		if(diagonal_coefficients.size() != nb_joints)
		{
			throw Exceptions::IncompatibleSizeException("The provided diagonal coefficients do not correspond to the expected size of " + std::to_string(nb_joints) + " elements");
		}
		Eigen::VectorXd diagonal = Eigen::VectorXd::Map(diagonal_coefficients.data(), nb_joints);
		this->gain_->set_value(diagonal.asDiagonal());
	}

	template<>
	inline void Linear<StateRepresentation::CartesianState>::set_gain(const Eigen::MatrixXd& gain_matrix)
	{
		if(gain_matrix.rows() != 6 && gain_matrix.cols() != 6)
		{
			throw Exceptions::IncompatibleSizeException("The provided gain matrix do not have the expected size of 6x6 elements");
		}
		this->gain_->set_value(gain_matrix);
	}

	template<>
	inline void Linear<StateRepresentation::JointState>::set_gain(const Eigen::MatrixXd& gain_matrix)
	{
		int nb_joints = this->get_attractor().get_size();
		if(gain_matrix.rows() != nb_joints && gain_matrix.cols() != nb_joints)
		{
			throw Exceptions::IncompatibleSizeException("The provided gain matrix do not have the expected size of " + std::to_string(nb_joints) + "x" + std::to_string(nb_joints) + " elements");
		}
		this->gain_->set_value(gain_matrix);
	}

	template<>
	Linear<StateRepresentation::CartesianState>::Linear(const StateRepresentation::CartesianState& attractor, double iso_gain):
	DynamicalSystem<StateRepresentation::CartesianState>(),
	attractor_(std::make_shared<StateRepresentation::Parameter<StateRepresentation::CartesianState>>(StateRepresentation::Parameter<StateRepresentation::CartesianPose>("attractor", attractor))),
	gain_(std::make_shared<StateRepresentation::Parameter<Eigen::MatrixXd>>("gain"))
	{
		this->set_gain(iso_gain);
	}

	template<>
	Linear<StateRepresentation::JointState>::Linear(const StateRepresentation::JointState& attractor, double iso_gain):
	DynamicalSystem<StateRepresentation::JointState>(),
	attractor_(std::make_shared<StateRepresentation::Parameter<StateRepresentation::JointState>>(StateRepresentation::Parameter<StateRepresentation::JointPositions>("attractor", attractor))),
	gain_(std::make_shared<StateRepresentation::Parameter<Eigen::MatrixXd>>("gain"))
	{
		this->set_gain(iso_gain);
	}

	template<>
	Linear<StateRepresentation::CartesianState>::Linear(const StateRepresentation::CartesianState& attractor, const std::vector<double>& diagonal_coefficients):
	DynamicalSystem<StateRepresentation::CartesianState>(),
	attractor_(std::make_shared<StateRepresentation::Parameter<StateRepresentation::CartesianState>>(StateRepresentation::Parameter<StateRepresentation::CartesianPose>("attractor", attractor))),
	gain_(std::make_shared<StateRepresentation::Parameter<Eigen::MatrixXd>>("gain"))
	{
		this->set_gain(diagonal_coefficients);
	}

	template<>
	Linear<StateRepresentation::JointState>::Linear(const StateRepresentation::JointState& attractor, const std::vector<double>& diagonal_coefficients):
	DynamicalSystem<StateRepresentation::JointState>(),
	attractor_(std::make_shared<StateRepresentation::Parameter<StateRepresentation::JointState>>(StateRepresentation::Parameter<StateRepresentation::JointPositions>("attractor", attractor))),
	gain_(std::make_shared<StateRepresentation::Parameter<Eigen::MatrixXd>>("gain"))
	{
		this->set_gain(diagonal_coefficients);
	}

	template<>
	Linear<StateRepresentation::CartesianState>::Linear(const StateRepresentation::CartesianState& attractor, const Eigen::MatrixXd& gain_matrix):
	DynamicalSystem<StateRepresentation::CartesianState>(),
	attractor_(std::make_shared<StateRepresentation::Parameter<StateRepresentation::CartesianState>>(StateRepresentation::Parameter<StateRepresentation::CartesianPose>("attractor", attractor))),
	gain_(std::make_shared<StateRepresentation::Parameter<Eigen::MatrixXd>>("gain"))
	{
		this->set_gain(gain_matrix);
	}

	template<>
	Linear<StateRepresentation::JointState>::Linear(const StateRepresentation::JointState& attractor, const Eigen::MatrixXd& gain_matrix):
	DynamicalSystem<StateRepresentation::JointState>(),
	attractor_(std::make_shared<StateRepresentation::Parameter<StateRepresentation::JointState>>(StateRepresentation::Parameter<StateRepresentation::JointPositions>("attractor", attractor))),
	gain_(std::make_shared<StateRepresentation::Parameter<Eigen::MatrixXd>>("gain"))
	{
		this->set_gain(gain_matrix);
	}

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
