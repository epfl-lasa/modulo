/**
 * @class JacobianMatrix
 * @brief Class to define a robot Jacobian matrix
 * @author Baptiste Busch
 * @date 2019/09/09
 */

#ifndef STATEREPRESENTATION_ROBOT_JACOBIANMATRIX_H_
#define STATEREPRESENTATION_ROBOT_JACOBIANMATRIX_H_

#include <eigen3/Eigen/Core>
#include "state_representation/State.hpp"
#include "state_representation/Cartesian/CartesianVelocity.hpp"
#include "state_representation/Joint/JointVelocities.hpp"
#include "state_representation/Exceptions/IncompatibleSizeException.hpp"

using namespace StateRepresentation::Exceptions;

namespace StateRepresentation 
{
	class CartesianVelocity;

	class JointVelocities;

	class JacobianMatrix: public State
	{
	private:
		std::vector<std::string> joint_names; ///< names of the joints
		unsigned int nb_rows; ///< number of rows
		unsigned int nb_cols; ///< number of columns
		Eigen::MatrixXd data; ///< internal storage of the jacobian matrix

	public:
		/**
	 	 * @brief Constructor with name and number of joints provided
	 	 * @brief name the name of the robot associated to
	 	 * @brief nb_joints the number of joints
	     */
		explicit JacobianMatrix(const std::string& robot_name, unsigned int nb_joints);

		/**
	 	 * @brief Constructor with name and list of joint names provided
	 	 * @brief name the name of the robot associated to
	 	 * @brief joint_names the list of joint names
	     */
		explicit JacobianMatrix(const std::string& robot_name, const std::vector<std::string>& joint_names);

		/**
	 	 * @brief Constructor with name and jacobian matrix as eigen matrix
	 	 * @brief name the name of the robot associated to
	 	 * @brief data the value of the jacobian matrix
	     */
		explicit JacobianMatrix(const std::string& robot_name, const Eigen::MatrixXd& data);

		/**
	 	 * @brief Constructor with name, list of joint names and jacobian matrix as eigen matrix
	 	 * @brief name the name of the robot associated to
	 	 * @brief joint_names the list of joint names
	 	 * @brief data the value of the jacobian matrix
	     */
		explicit JacobianMatrix(const std::string& robot_name, const std::vector<std::string>& joint_names, const Eigen::MatrixXd& data);

		/**
	 	 * @brief Copy constructor of a JacobianMatrix
	     */
		JacobianMatrix(const JacobianMatrix& jacobian);

		/**
	 	 * @brief Getter of the nb_rows from the attributes
	     */
		unsigned int get_nb_rows() const;

		/**
		 * @brief Setter of the nb_rows
		 */
		void set_nb_rows(unsigned int nb_rows);

		/**
		 * @brief Setter of the nb_cols
		 */
		void set_nb_cols(unsigned int nb_cols);

		/**
	 	 * @brief Getter of the nb_cols from the attributes
	     */
		unsigned int get_nb_cols() const;

		/**
	 	 * @brief Getter of the names attribute
	     */
		const std::vector<std::string>& get_joint_names() const;

		/**
	 	 * @brief Setter of the names attribute from the number of joints
	     */
		void set_joint_names(unsigned int nb_joints);

		/**
	 	 * @brief Setter of the names attribute
	     */
		void set_joint_names(const std::vector<std::string>& names);

		/**
	 	 * @brief Getter of the data attribute
	     */
		const Eigen::MatrixXd& get_data() const;

		/**
	 	 * @brief Setter of the data attribute
	     */
		void set_data(const Eigen::MatrixXd& data);

		/**
	 	 * @brief Check if the jacobian matrix is compatible for operations with the state given as argument
	 	 * @param state the state to check compatibility with
	     */
		bool is_compatible(const State& state) const;

		/**
	 	 * @brief Initialize the matrix to a zero value
	     */
		void initialize();

		/**
		 * @brief Return the transpose of the jacobian matrix
		 * @return the JacobianMatrix transposed
		 */
		const JacobianMatrix transpose();

		/**
	 	 * @brief Overload the * operator with an non specific matrix
	 	 * @param matrix the vector to multiply with
	 	 * @return the vector multiply by the jacobian matrix
	     */
		const Eigen::MatrixXd operator*(const Eigen::MatrixXd& matrix) const;

		/**
	 	 * @brief Overload the * operator with a JointVelocities
	 	 * @param dq the joint velocity to multiply with
	 	 * @return this result into the CartesianVelocity of the end effector
	 	 * the name of the output CartesianVelocity will be "robot"_end_effector and
	 	 * the reference frame will be "robot"_base 
	     */
		const CartesianVelocity operator*(const JointVelocities& dq) const;

		/**
	 	 * @brief Solve the system X = inv(J)*M to obtain X which is more efficient than multiplying with the pseudo-inverse
	 	 * @param matrix the matrix to solve the system with
	 	 * @return result of X = J.solve(M) from Eigne decomposition
	     */
		const Eigen::MatrixXd solve(const Eigen::MatrixXd& matrix) const;

		/**
	 	 * @brief Solve the system dX = J*dq to obtain dq which is more efficient than multiplying with the pseudo-inverse
	 	 * @param dX the cartesian velocity to multiply with
	 	 * @return this result into a JointVelocities
	     */
		const JointVelocities solve(const CartesianVelocity& dX) const;

		/**
	 	 * @brief Overload the * operator with a CartesianVelocity. This is equivalent to using the solve function
	 	 * @param dX the cartesian velocity to multiply with
	 	 * @return this result into a JointVelocities
	     */
		const JointVelocities operator*(const CartesianVelocity& dX) const;

		/**
		 * @brief Overload the () operator in a non const fashion to modify the value at given (row, col)
		 * @param row the index of the row
		 * @param the index of the column
		 * @return the reference to the value at the given row and column
		 */
		double& operator()(unsigned int row, unsigned int col);

		/**
		 * @brief Overload the () operator const fashion to access the value at given (row, col)
		 * @param row the index of the row
		 * @param the index of the column
		 * @return the const reference to the value at the given row and column
		 */
		const double& operator()(unsigned int row, unsigned int col) const;

		/**
		 * @brief Return a copy of the JointPositions
		 * @return the copy
		 */
		const JacobianMatrix copy() const;

		/**
	 	 * @brief Overload the ostream operator for printing
	 	 * @param os the ostream to append the string representing the matrix to
	 	 * @param matrix the matrix to print
	 	 * @return the appended ostream
	     */
		friend std::ostream& operator<<(std::ostream& os, const JacobianMatrix& matrix);
	};

	inline unsigned int JacobianMatrix::get_nb_rows() const
	{
		return this->nb_rows;
	}

	inline unsigned int JacobianMatrix::get_nb_cols() const
	{
		return this->nb_cols;
	}

	inline void JacobianMatrix::set_nb_rows(unsigned int nb_rows)
	{
		this->nb_rows = nb_rows;
	}

	inline void JacobianMatrix::set_nb_cols(unsigned int nb_cols)
	{
		this->nb_cols = nb_cols;
	}

	inline const std::vector<std::string>& JacobianMatrix::get_joint_names() const
	{
		return this->joint_names;
	}

	inline void JacobianMatrix::set_joint_names(unsigned int nb_joints)
	{
		this->joint_names.resize(nb_joints);
		for(unsigned int i=0; i<nb_joints; ++i)
		{
			this->joint_names[i] = "joint" + std::to_string(i);
		}
		this->initialize();
	}

	inline void JacobianMatrix::set_joint_names(const std::vector<std::string>& joint_names)
	{
		this->joint_names = joint_names;
		this->initialize();
	}

	inline const Eigen::MatrixXd& JacobianMatrix::get_data() const
	{
		return this->data;
	}

	inline void JacobianMatrix::set_data(const Eigen::MatrixXd& data)
	{
		if(this->get_nb_rows() != data.rows() || this->get_nb_cols() != data.cols()) throw IncompatibleSizeException("Input matrix is of incorrect size");
		this->set_filled();
		this->data = data;
	}

	inline bool JacobianMatrix::is_compatible(const State& state) const
	{
		bool compatible = false;
		if(state.get_type() == "JointState")
		{
			compatible = (this->get_name() == state.get_name()) && (this->get_nb_cols() == static_cast<const JointState&>(state).get_size());
			if(compatible)
			{
				for(unsigned int i=0; i<this->get_nb_cols(); ++i) compatible = (compatible && this->joint_names[i] == static_cast<const JointState&>(state).get_names()[i]);
			}
		}
		// there is no possibilities to check that a correct frame associated to the robot is sent
		else if(state.get_type() == "CartesianState")
		{
			compatible = true;
		}
		return compatible;
	}

	inline double& JacobianMatrix::operator()(unsigned int row, unsigned int col)
	{
		if(row > this->get_nb_rows()) throw std::out_of_range("Given row is out of range: number of rows = " + this->get_nb_rows());
		if(col > this->get_nb_cols()) throw std::out_of_range("Given column is out of range: number of columns = " + this->get_nb_cols());
		return this->data(row, col);
	}

	inline const double& JacobianMatrix::operator()(unsigned int row, unsigned int col) const
	{
		if(row > this->get_nb_rows()) throw std::out_of_range("Given row is out of range: number of rows = " + this->get_nb_rows());
		if(col > this->get_nb_cols()) throw std::out_of_range("Given column is out of range: number of columns = " + this->get_nb_cols());
		return this->data(row, col);
	}
}
#endif