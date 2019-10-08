#include "state_representation/Robot/JacobianMatrix.hpp"
#include "state_representation/Exceptions/IncompatibleStatesException.hpp"
#include "state_representation/Exceptions/EmptyStateException.hpp"

namespace StateRepresentation 
{
	JacobianMatrix::JacobianMatrix(const std::string& robot_name, unsigned int nb_joints):
	State("JacobianMatrix", robot_name), nb_rows(6), nb_cols(nb_joints)
	{
		this->set_joint_names(nb_joints);
	}

	JacobianMatrix::JacobianMatrix(const std::string& robot_name, const std::vector<std::string>& joint_names):
	State("JacobianMatrix", robot_name), nb_rows(6), nb_cols(joint_names.size())
	{
		this->set_joint_names(joint_names);
	}

	JacobianMatrix::JacobianMatrix(const std::string& robot_name, const Eigen::MatrixXd& data):
	State("JacobianMatrix", robot_name), nb_rows(data.rows()), nb_cols(data.cols())
	{
		this->set_joint_names(this->get_nb_cols());
		this->set_data(data);
	}

	JacobianMatrix::JacobianMatrix(const std::string& robot_name, const std::vector<std::string>& joint_names, const Eigen::MatrixXd& data):
	State("JacobianMatrix", robot_name), nb_rows(data.rows()), nb_cols(joint_names.size())
	{
		this->set_joint_names(joint_names);
		this->set_data(data);
	}

	JacobianMatrix::JacobianMatrix(const JacobianMatrix& jacobian):
	State(jacobian), joint_names(jacobian.joint_names), data(jacobian.data)
	{}

	void JacobianMatrix::initialize()
	{
		this->State::initialize();
		this->data.resize(this->nb_rows, this->nb_cols);
		this->data.setZero();
	}

	const JacobianMatrix JacobianMatrix::transpose()
	{
		JacobianMatrix result(*this);
		result.set_nb_cols(this->get_nb_rows());
		result.set_nb_rows(this->get_nb_cols());
		result.set_data(result.get_data().transpose());
		return result;
	}

	const Eigen::MatrixXd JacobianMatrix::operator*(const Eigen::MatrixXd& matrix) const
	{
		if(this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
		if(matrix.rows() != this->get_nb_cols()) throw IncompatibleSizeException("Input matrix is of incorrect size");
		return this->get_data() * matrix;
	}

	const CartesianTwist JacobianMatrix::operator*(const JointVelocities& dq) const
	{
		if(dq.is_empty()) throw EmptyStateException(dq.get_name() + " state is empty");
		if(!this->is_compatible(dq)) throw IncompatibleStatesException("The JacobianMatrix and the input JointVelocities are incompatible"o);
		Eigen::Matrix<double, 6, 1> twist = (*this) * dq.get_velocities();
		// crate a CartesianTwist with name "robot"_end_effector and reference frame "robot"_base
		CartesianTwist result(this->get_name() + "_end_effector", twist, this->get_name() + "_base");
		return result;
	}

	const Eigen::MatrixXd JacobianMatrix::solve(const Eigen::MatrixXd& matrix) const
	{
		if(this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
		if(this->get_nb_rows() != matrix.rows()) throw IncompatibleSizeException("Input matrix is of incorrect size");
		return this->get_data().colPivHouseholderQr().solve(matrix);
	}

	const JointVelocities JacobianMatrix::solve(const CartesianTwist& dX) const
	{
		if(dX.is_empty()) throw EmptyStateException(dX.get_name() + " state is empty");
		// extract the velocity as a 6d vector
		auto twist = dX.get_twist();
		// this use the solve operation instead of using the inverse or pseudo-inverse of the jacobian
		Eigen::VectorXd joint_velocities = (*this).solve(twist);
		// return a JointVelocities state
		JointVelocities result(this->get_name(), this->get_joint_names(), joint_velocities);
		return result;
	}

	const JointVelocities JacobianMatrix::operator*(const CartesianTwist& dX) const
	{
		return this->solve(dX);
	}

	const JacobianMatrix JacobianMatrix::copy() const
	{
		JacobianMatrix result(*this);
		return result;
	}

	std::ostream& operator<<(std::ostream& os, const JacobianMatrix& matrix)
	{
		if(matrix.is_empty())
		{
			os << "Empty JacobianMatrix";
		}
		else
		{
			os << matrix.get_name() << " JacobianMatrix" << std::endl;
			os << "joint names: [";
			for(auto& n:matrix.get_joint_names()) os << n << ", ";
			os << "]" << std::endl;
			os << "number of rows: " << matrix.get_nb_rows() << std::endl;
			os << "number of columns: " << matrix.get_nb_cols() << std::endl;
			for(unsigned int i = 0; i < matrix.get_nb_rows(); ++i)
			{
				os << "| " << matrix(i,0);
				for(unsigned int j = 1; j < matrix.get_nb_cols(); ++j)
				{
					os << ", " << matrix(i,j);
				}
				os << " |" << std::endl;
			}
		}
		return os;
	}
}
