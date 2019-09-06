#include "state_representation/Cartesian/CartesianPose.hpp"
#include "state_representation/Exceptions/IncompatibleReferenceFramesException.hpp"
#include "state_representation/Exceptions/IncompatibleStatesException.hpp"
#include "state_representation/Exceptions/EmptyStateException.hpp"

using namespace StateRepresentation::Exceptions;

namespace StateRepresentation 
{
	CartesianPose::CartesianPose()
	{}

	CartesianPose::CartesianPose(const std::string& name, const std::string& reference):
	CartesianState(name, reference)
	{}

	CartesianPose::CartesianPose(const std::string& name, const Eigen::Vector3d& position, const std::string& reference):
	CartesianState(name, reference)
	{
		this->set_position(position);
	}

	CartesianPose::CartesianPose(const std::string& name, const double& x, const double& y, const double& z, const std::string& reference):
	CartesianState(name, reference)
	{
		this->set_position(x,y,z);
	}

	CartesianPose::CartesianPose(const std::string& name, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, const std::string& reference):
	CartesianState(name, reference)
	{
		this->set_position(position);
		this->set_orientation(orientation);
	}

	CartesianPose::CartesianPose(const CartesianPose& p):
	CartesianState(p)
	{}

	CartesianPose::CartesianPose(const CartesianState& s):
	CartesianState(s)
	{}

	CartesianPose::CartesianPose(const CartesianVelocity& v):
	CartesianState(std::chrono::seconds(1) * v)
	{}

	CartesianPose& CartesianPose::operator*=(const CartesianPose& p)
	{
		// sanity check
		if(this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
		if(p.is_empty()) throw EmptyStateException(p.get_name() + " state is empty");
		if(this->get_name() != p.get_reference_frame()) throw IncompatibleReferenceFramesException("Expected " + this->get_name() + ", got " + p.get_reference_frame());
		// operation
		this->set_position(this->get_position() + this->get_orientation() * p.get_position());
		this->set_orientation(this->get_orientation() * p.get_orientation());
		this->set_name(p.get_name());
		return (*this);
	}

	const CartesianPose CartesianPose::operator*(const CartesianPose& p) const
	{
		CartesianPose result(*this);
		result *= p;
		return result;
	}

	const CartesianState CartesianPose::operator*(const CartesianState& s) const
	{
		// sanity check
		if(this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
		if(s.is_empty()) throw EmptyStateException(s.get_name() + " state is empty");
		if(this->get_name() != s.get_reference_frame()) throw IncompatibleReferenceFramesException("Expected " + this->get_name() + ", got " + s.get_reference_frame());
		// operation
		CartesianState result(*this);
		result.set_position(this->get_position() + this->get_orientation() * s.get_position());
		result.set_orientation(this->get_orientation() * s.get_orientation());
		result.set_linear_velocity(*this * s.get_linear_velocity());
		result.set_angular_velocity(*this * s.get_angular_velocity());
		result.set_linear_acceleration(*this * s.get_linear_acceleration());
		result.set_angular_acceleration(*this * s.get_angular_acceleration());
		result.set_force(*this * s.get_force());
		result.set_torque(*this * s.get_torque());
		result.set_name(s.get_name());
		return result;
	}

	const Eigen::Vector3d CartesianPose::operator*(const Eigen::Vector3d& v) const
	{
		return this->get_orientation() * v + this->get_position();
	}

	CartesianPose& CartesianPose::operator+=(const CartesianPose& p)
	{
		// sanity check
		if(this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
		if(p.is_empty()) throw EmptyStateException(p.get_name() + " state is empty");
		if(!this->is_compatible(p)) throw IncompatibleStatesException("The two states do not have the same name nor reference frame");
		// operation
		this->set_position(this->get_position() + p.get_position());
		this->set_orientation(this->get_orientation() * p.get_orientation());
		return (*this);
	}

	const CartesianPose CartesianPose::operator+(const CartesianPose& p) const
	{
		CartesianPose result(*this);
		result += p;
		return result;
	}

	CartesianPose& CartesianPose::operator-=(const CartesianPose& p)
	{
		// sanity check
		if(this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
		if(p.is_empty()) throw EmptyStateException(p.get_name() + " state is empty");
		if(!this->is_compatible(p)) throw IncompatibleStatesException("The two states do not have the same name nor reference frame");
		// operation
		this->set_position(this->get_position() - p.get_position());
		this->set_orientation(this->get_orientation() * p.get_orientation().conjugate());
		return (*this);
	}

	const CartesianPose CartesianPose::operator-(const CartesianPose& p) const
	{
		CartesianPose result(*this);
		result -= p;
		return result;
	}

	CartesianPose& CartesianPose::operator*=(double lambda)
	{
		// sanity check
		if(this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
		// operation
		this->set_position(lambda * this->get_position());
		this->set_orientation(Eigen::Quaterniond(lambda * this->get_orientation().coeffs()));
		return (*this);
	}

	const CartesianPose CartesianPose::operator*(double lambda) const
	{
		CartesianPose result(*this);
		result *= lambda;
		return result;
	}

	const CartesianPose CartesianPose::inverse() const
	{
		CartesianPose result(*this);
		result.set_orientation(this->get_orientation().conjugate());
		result.set_position(result.get_orientation() * (- this->get_position()));
		// inverse name and reference frame
		std::string ref = result.get_reference_frame();
		std::string name = result.get_name();
		result.set_reference_frame(name);
		result.set_name(ref);
		return result;
	}

	std::ostream& operator<<(std::ostream& os, const CartesianPose& pose) 
	{
		if(pose.is_empty())
		{
			os << "Empty CartesianPose";
		}
		else
		{
			os << pose.get_name() << " CartesianPose expressed in " << pose.get_reference_frame() << " frame" << std::endl;
	  		os << "position: (" << pose.get_position()(0) << ", ";
	  		os << pose.get_position()(1) << ", ";
	  		os << pose.get_position()(2) << ")" << std::endl;
	  		os << "orientation: (" <<pose.get_orientation().w() << ", ";
	  		os << pose.get_orientation().x() << ", ";
	  		os << pose.get_orientation().y() << ", ";
	  		os << pose.get_orientation().z() << ")";
	  		Eigen::AngleAxisd axis_angle(pose.get_orientation());
	  		os << " <=> theta: " << axis_angle.angle() << ", ";
	  		os << "axis: (" << axis_angle.axis()(0) << ", ";
	  		os << axis_angle.axis()(1) << ", ";
	  		os << axis_angle.axis()(2) << ")";
	  	}
  		return os;
	}

	const CartesianPose operator*(double lambda, const CartesianPose& pose)
	{
		return pose * lambda;
	}

	const CartesianVelocity operator/(const CartesianPose& pose, const std::chrono::milliseconds& dt)
	{
		// sanity check
		if(pose.is_empty()) throw EmptyStateException(pose.get_name() + " state is empty");
		// operations
		CartesianVelocity velocity(pose.get_name(), pose.get_reference_frame());
		// convert the period to a double with the second as reference
		double period = std::chrono::milliseconds(dt).count();
		period /= 1000.;
		// set linear velocity
		velocity.set_linear_velocity(pose.get_position() / period);
		// set angular velocity from the log of the quaternion error
		Eigen::AngleAxisd axis_angle(pose.get_orientation());
		Eigen::Vector3d rotation = (axis_angle.angle() * axis_angle.axis()) / 2;
		velocity.set_angular_velocity(rotation / period);
		return velocity;
	}
}