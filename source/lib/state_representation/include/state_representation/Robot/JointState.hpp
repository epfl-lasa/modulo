/**
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#pragma once

#include <eigen3/Eigen/Core>
#include <vector>
#include <string>
#include <iostream>
#include <math.h>
#include "state_representation/State.hpp"
#include "state_representation/Exceptions/IncompatibleSizeException.hpp"

using namespace StateRepresentation::Exceptions;

namespace StateRepresentation 
{
	/**
	 * @class JointState
	 * @brief Class to define a state in joint space
	 */
	class JointState: public State
	{
	private:
		std::vector<std::string> names; ///< names of the joints
		Eigen::VectorXd positions; ///< joints positions
		Eigen::VectorXd velocities; ///< joints velocities
		Eigen::VectorXd accelerations; ///< joints accelerations
		Eigen::VectorXd torques; ///< joints torques

	public:
		/**
	 	 * @brief Empty constructor for a JointState
	     */
		explicit JointState();

		/**
	 	 * @brief Constructor with name and number of joints provided
	 	 * @brief name the name of the state
	 	 * @brief nb_joints the number of joints for initialization
	     */
		explicit JointState(const std::string& robot_name, unsigned int nb_joints=0);

		/**
	 	 * @brief Constructor with name and list of joint names provided
	 	 * @brief name the name of the state
	 	 * @brief joint_names list of joint names
	     */
		explicit JointState(const std::string& robot_name, const std::vector<std::string>& joint_names);

		/**
	 	 * @brief Copy constructor of a JointState
	     */
		JointState(const JointState& state);

		/**
		 * @brief Copy assignement operator that have to be defined to the custom assignement operator
		 * @param state the state with value to assign
		 * @return reference to the current state with new values
		 */
		JointState& operator=(const JointState& state);

		/**
	 	 * @brief Getter of the size from the attributes
	     */
		unsigned int get_size() const;

		/**
	 	 * @brief Getter of the names attribute
	     */
		const std::vector<std::string>& get_names() const;

		/**
	 	 * @brief Setter of the names attribute from the number of joints
	     */
		void set_names(unsigned int nb_joints);

		/**
	 	 * @brief Setter of the names attribute
	     */
		void set_names(const std::vector<std::string>& names);

		/**
	 	 * @brief Getter of the posistions attribute
	     */
		const Eigen::VectorXd& get_positions() const;

		/**
	 	 * @brief Setter of the posistions attribute
	     */
		void set_positions(const Eigen::VectorXd& positions);

		/**
	 	 * @brief Setter of the posistions attribute
	     */
		void set_positions(const std::vector<double>& positions);

		/**
	 	 * @brief Getter of the velocities attribute
	     */
		const Eigen::VectorXd& get_velocities() const;

		/**
	 	 * @brief Setter of the velocities attribute
	     */
		void set_velocities(const Eigen::VectorXd& velocities);

		/**
	 	 * @brief Getter of the accelerations attribute
	     */
		const Eigen::VectorXd& get_accelerations() const;

		/**
	 	 * @brief Setter of the accelerations attribute
	     */
		void set_accelerations(const Eigen::VectorXd& accelerations);

		/**
	 	 * @brief Getter of the torques attribute
	     */
		const Eigen::VectorXd& get_torques() const;

		/**
	 	 * @brief Setter of the torques attribute
	     */
		void set_torques(const Eigen::VectorXd& torques);

		/**
	 	 * @brief Check if the state is compatible for operations with the state given as argument
	 	 * @param state the state to check compatibility with
	     */
		bool is_compatible(const State& state) const;

		/**
	 	 * @brief Initialize the State to a zero value
	     */
		void initialize();

		/**
	 	 * @brief Set the State to a zero value
	     */
		void set_zero();

		/**
		 * @brief Return a copy of the JointState
		 * @return the copy
		 */
		const JointState copy() const;

		/**
	 	 * @brief Overload the += operator
	 	 * @param js JointState to add
	 	 * @return the current JointState added the JointState given in argument
	     */
		JointState& operator+=(const JointState& js);

		/**
	 	 * @brief Overload the + operator
	 	 * @param js JointState to add
	 	 * @return the current JointState added the JointState given in argument
	     */
		const JointState operator+(const JointState& js) const;

		/**
	 	 * @brief Overload the -= operator
	 	 * @param js JointState to substract
	 	 * @return the current JointState substracted the JointState given in argument
	     */
		JointState& operator-=(const JointState& js);

		/**
	 	 * @brief Overload the - operator
	 	 * @param js JointState to substract
	 	 * @return the current JointState substracted the JointState given in argument
	     */
		const JointState operator-(const JointState& js) const;

		/**
	 	 * @brief Overload the -= operator with a matrix of gains
	 	 * @param lambda the matrix to multiply with
	 	 * @return the JointState multiplied by lambda
	     */
		JointState& operator*=(const Eigen::MatrixXd& lambda);

		/**
	 	 * @brief Overload the ostream operator for printing
	 	 * @param os the ostream to append the string representing the state
	 	 * @param state the state to print
	 	 * @return the appended ostream
	     */
		friend std::ostream& operator<<(std::ostream& os, const JointState& state);

		/**
	 	 * @brief Overload the * operator with a scalar
	 	 * @param lambda the scalar to multiply with
	 	 * @return the JointState provided multiply by lambda
	     */
		friend const JointState operator*(double lambda, const JointState& state);

		/**
	 	 * @brief Overload the * operator with a matrix of gains
	 	 * @param lambda the matrix to multiply with
	 	 * @return the JointState provided multiply by lambda
	     */
		friend const JointState operator*(const Eigen::MatrixXd& lambda, const JointState& state);

		/**
		 * @brief Return the joint state as a std vector of floats
		 * @return std::vector<float> the joint vector as a std vector
		 */
		virtual const std::vector<double> to_std_vector() const;

		/**
		 * @brief Set the value from a std vector
		 * @param value the value as a std vector
		 */
		virtual void from_std_vector(const std::vector<double>& value);
	};

	inline JointState& JointState::operator=(const JointState& state)
	{
		State::operator=(state);
		this->set_names(state.get_names());
		this->set_positions(state.get_positions());
		this->set_velocities(state.get_velocities());
		this->set_accelerations(state.get_accelerations());
		this->set_torques(state.get_torques());
		return (*this);
	}

	inline bool JointState::is_compatible(const State& state) const
	{
		bool compatible = this->State::is_compatible(state);
		compatible = compatible && (this->names.size() == static_cast<const JointState&>(state).names.size());
		if(compatible)
		{
			for(unsigned int i=0; i<this->names.size(); ++i) compatible = (compatible && this->names[i] == static_cast<const JointState&>(state).names[i]);
		}
		return compatible;
	}

	inline unsigned int JointState::get_size() const
	{
		return this->names.size();
	}

	inline const std::vector<std::string>& JointState::get_names() const
	{
		return this->names;
	}

	inline void JointState::set_names(unsigned int nb_joints)
	{
		this->names.resize(nb_joints);
		for(unsigned int i=0; i<nb_joints; ++i)
		{
			this->names[i] = "joint" + std::to_string(i);
		}
		this->initialize();
	}

	inline void JointState::set_names(const std::vector<std::string>& names)
	{
		this->names = names;
		this->initialize();
	}

	inline const Eigen::VectorXd& JointState::get_positions() const
	{
		return this->positions;
	}

	inline void JointState::set_positions(const Eigen::VectorXd& positions)
	{
		if(positions.size() != this->get_size()) throw IncompatibleSizeException("Input vector is of incorrect size: expected " + std::to_string(this->get_size()) + ", given " + std::to_string(positions.size()));
		this->set_filled();
		// positions are angles between -pi and pi
		this->positions = positions.unaryExpr([](double x){return atan2(sin(x), cos(x));});
	}

	inline void JointState::set_positions(const std::vector<double>& positions)
	{
		if(positions.size() != this->get_size()) throw IncompatibleSizeException("Input vector is of incorrect size: expected " + std::to_string(this->get_size()) + ", given " + std::to_string(positions.size()));
		this->set_filled();
		this->positions = Eigen::VectorXd::Map(positions.data(), positions.size());
		// positions are angles between -pi and pi
		this->positions = this->positions.unaryExpr([](double x){return atan2(sin(x), cos(x));});
	}

	inline const Eigen::VectorXd& JointState::get_velocities() const
	{
		return this->velocities;
	}

	inline void JointState::set_velocities(const Eigen::VectorXd& velocities)
	{
		if(velocities.size() != this->get_size()) throw IncompatibleSizeException("Input vector is of incorrect size: expected " + std::to_string(this->get_size()) + ", given " + std::to_string(velocities.size()));
		this->set_filled();
		this->velocities = velocities;
	}

	inline const Eigen::VectorXd& JointState::get_accelerations() const
	{
		return this->accelerations;
	}

	inline void JointState::set_accelerations(const Eigen::VectorXd& accelerations)
	{
		if(accelerations.size() != this->get_size()) throw IncompatibleSizeException("Input vector is of incorrect size: expected " + std::to_string(this->get_size()) + ", given " + std::to_string(accelerations.size()));
		this->set_filled();
		this->accelerations = accelerations;
	}

	inline const Eigen::VectorXd& JointState::get_torques() const
	{
		return this->torques;
	}

	inline void JointState::set_torques(const Eigen::VectorXd& torques)
	{
		if(torques.size() != this->get_size()) throw IncompatibleSizeException("Input vector is of incorrect size: expected " + std::to_string(this->get_size()) + ", given " + std::to_string(torques.size()));
		this->set_filled();
		this->torques = torques;
	}
}
