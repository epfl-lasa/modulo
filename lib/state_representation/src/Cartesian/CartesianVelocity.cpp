#include "state_representation/Cartesian/CartesianVelocity.hpp"
#include "state_representation/Exceptions/IncompatibleStatesException.hpp"
#include "state_representation/Exceptions/EmptyStateException.hpp"

using namespace StateRepresentation::Exceptions;

namespace StateRepresentation 
{
	CartesianVelocity::CartesianVelocity(const std::string& name, const std::string& reference):
	CartesianState(name, reference)
	{}

	CartesianVelocity::CartesianVelocity(const std::string& name, const Eigen::Vector3d& linear_velocity, const std::string& reference):
	CartesianState(name, reference)
	{
		this->set_linear_velocity(linear_velocity);
	}

	CartesianVelocity::CartesianVelocity(const std::string& name, const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& angular_velocity, const std::string& reference):
	CartesianState(name, reference)
	{
		this->set_linear_velocity(linear_velocity);
		this->set_angular_velocity(angular_velocity);
	}

	CartesianVelocity::CartesianVelocity(const CartesianVelocity& p):
	CartesianState(p)
	{}

	CartesianVelocity::CartesianVelocity(const CartesianState& s):
	CartesianState(s)
	{}

	CartesianVelocity::CartesianVelocity(const CartesianPose& p):
	CartesianState(p / std::chrono::seconds(1))
	{}

	CartesianVelocity& CartesianVelocity::operator+=(const CartesianVelocity& v)
	{
		// sanity check
		if(this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
		if(v.is_empty()) throw EmptyStateException(v.get_name() + " state is empty");
		if(!this->is_compatible(v)) throw IncompatibleStatesException("The two states do not have the same name nor reference frame");
		// operation
		this->set_linear_velocity(this->get_linear_velocity() + v.get_linear_velocity());
		this->set_angular_velocity(this->get_angular_velocity() + v.get_angular_velocity());
		return (*this);
	}

	const CartesianVelocity CartesianVelocity::operator+(const CartesianVelocity& v) const
	{
		CartesianVelocity result(*this);
		result += v;
		return result;
	}

	CartesianVelocity& CartesianVelocity::operator-=(const CartesianVelocity& v)
	{
		// sanity check
		if(this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
		if(v.is_empty()) throw EmptyStateException(v.get_name() + " state is empty");
		if(!this->is_compatible(v)) throw IncompatibleStatesException("The two states do not have the same name nor reference frame");
		// operation
		this->set_linear_velocity(this->get_linear_velocity() - v.get_linear_velocity());
		this->set_angular_velocity(this->get_angular_velocity() - v.get_angular_velocity());
		return (*this);
	}

	const CartesianVelocity CartesianVelocity::operator-(const CartesianVelocity& v) const
	{
		CartesianVelocity result(*this);
		result -= v;
		return result;
	}

	void CartesianVelocity::operator=(const CartesianState& s)
	{
		// sanity check
		if(s.is_empty()) throw EmptyStateException(s.get_name() + " state is empty");
		// operation
		this->set_name(s.get_name());
		this->set_reference_frame(s.get_reference_frame());
		this->set_linear_velocity(s.get_linear_velocity());
		this->set_angular_velocity(s.get_angular_velocity());
	}

	CartesianVelocity& CartesianVelocity::operator*=(double lambda)
	{
		// sanity check
		if(this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
		// operation
		this->set_linear_velocity(lambda * this->get_linear_velocity());
		this->set_angular_velocity(lambda * this->get_angular_velocity());
		return (*this);
	}

	const CartesianVelocity CartesianVelocity::operator*(double lambda) const
	{
		CartesianVelocity result(*this);
		result *= lambda;
		return result;
	}

	std::ostream& operator<<(std::ostream& os, const CartesianVelocity& velocity) 
	{
		if(velocity.is_empty())
		{
			os << "Empty CartesianVelocity";
		}
		else
		{
			os << velocity.get_name() << " CartesianVelocity expressed in " << velocity.get_reference_frame() << " frame" << std::endl;
	  		os << "linear_velocity: (" << velocity.get_linear_velocity()(0) << ", ";
	  		os << velocity.get_linear_velocity()(1) << ", ";
	  		os << velocity.get_linear_velocity()(2) << ")" << std::endl;
	  		os << "angular_velocity: (" << velocity.get_angular_velocity()(0) << ", ";
	  		os << velocity.get_angular_velocity()(1) << ", ";
	  		os << velocity.get_angular_velocity()(2) << ")";
	  	}
  		return os;
	}

	const CartesianVelocity operator*(double lambda, const CartesianVelocity& velocity)
	{
		return velocity * lambda;
	}

	const CartesianPose operator*(const std::chrono::milliseconds& dt, const CartesianVelocity& velocity)
	{
		// sanity check
		if(velocity.is_empty()) throw EmptyStateException(velocity.get_name() + " state is empty");
		// operations
		CartesianPose displacement(velocity.get_name(), velocity.get_reference_frame());
		// convert the period to a double with the second as reference
		double period = std::chrono::milliseconds(dt).count();
		period /= 1000.;
		// multiply the velocity by this period value
		CartesianVelocity tmp = period * velocity;
		// convert the velocities into a displacement
		displacement.set_position(tmp.get_linear_velocity());
		Eigen::Quaterniond angular_velocity = Eigen::Quaterniond(0, tmp.get_angular_velocity()(0), tmp.get_angular_velocity()(1), tmp.get_angular_velocity()(2));
		Eigen::Quaterniond origin = Eigen::Quaterniond::Identity();
		displacement.set_orientation(Eigen::Quaterniond(origin.coeffs() + 0.5 * (origin * angular_velocity).coeffs()));
		return displacement;
	}

	const CartesianPose operator*(const CartesianVelocity& velocity, const std::chrono::milliseconds& dt)
	{
		return dt * velocity;
	}
}