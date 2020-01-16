#include "state_representation/Space/Cartesian/CartesianPose.hpp"
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

	CartesianPose::CartesianPose(const CartesianPose& pose):
	CartesianState(pose)
	{}

	CartesianPose::CartesianPose(const CartesianState& state):
	CartesianState(state)
	{}

	CartesianPose::CartesianPose(const CartesianTwist& twist):
	CartesianState(std::chrono::seconds(1) * twist)
	{}

	CartesianPose& CartesianPose::operator*=(const CartesianPose& pose)
	{
		// sanity check
		if(this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
		if(pose.is_empty()) throw EmptyStateException(pose.get_name() + " state is empty");
		if(this->get_name() != pose.get_reference_frame()) throw IncompatibleReferenceFramesException("Expected " + this->get_name() + ", got " + pose.get_reference_frame());
		// operation
		this->set_position(this->get_position() + this->get_orientation() * pose.get_position());
		this->set_orientation(this->get_orientation() * pose.get_orientation());
		this->set_name(pose.get_name());
		return (*this);
	}

	const CartesianPose CartesianPose::operator*(const CartesianPose& pose) const
	{
		CartesianPose result(*this);
		result *= pose;
		return result;
	}

	const CartesianState CartesianPose::operator*(const CartesianState& state) const
	{
		// sanity check
		if(this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
		if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty");
		if(this->get_name() != state.get_reference_frame()) throw IncompatibleReferenceFramesException("Expected " + this->get_name() + ", got " + state.get_reference_frame());
		// operation
		CartesianState result(*this);
		result.set_position(this->get_position() + this->get_orientation() * state.get_position());
		result.set_orientation(this->get_orientation() * state.get_orientation());
		result.set_linear_velocity(*this * state.get_linear_velocity());
		result.set_angular_velocity(*this * state.get_angular_velocity());
		result.set_linear_acceleration(*this * state.get_linear_acceleration());
		result.set_angular_acceleration(*this * state.get_angular_acceleration());
		result.set_force(*this * state.get_force());
		result.set_torque(*this * state.get_torque());
		result.set_name(state.get_name());
		return result;
	}

	const Eigen::Vector3d CartesianPose::operator*(const Eigen::Vector3d& vector) const
	{
          return this->get_orientation() * vector + this->get_position();
	}

	CartesianPose& CartesianPose::operator+=(const CartesianPose& pose)
	{
		// sanity check
		if(this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
		if(pose.is_empty()) throw EmptyStateException(pose.get_name() + " state is empty");
		if(!(this->get_reference_frame() == pose.get_reference_frame())) throw IncompatibleReferenceFramesException("The two states do not have the same reference frame");
		// operation
		this->set_position(this->get_position() + pose.get_position());
		Eigen::Quaterniond qres = this->get_orientation() * pose.get_orientation();
		if(this->get_orientation().dot(qres) < 0) qres = Eigen::Quaterniond(-qres.coeffs());
		this->set_orientation(qres);
		return (*this);
	}

	const CartesianPose CartesianPose::operator+(const CartesianPose& pose) const
	{
		CartesianPose result(*this);
		result += pose;
		return result;
	}

	CartesianPose& CartesianPose::operator-=(const CartesianPose& pose)
	{
		// sanity check
		if(this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
		if(pose.is_empty()) throw EmptyStateException(pose.get_name() + " state is empty");
		if(!(this->get_reference_frame() == pose.get_reference_frame())) throw IncompatibleReferenceFramesException("The two states do not have the same reference frame");
		// operation
		this->set_position(this->get_position() - pose.get_position());
		Eigen::Quaterniond qres = this->get_orientation() * pose.get_orientation().conjugate();
		if(this->get_orientation().dot(qres) < 0) qres = Eigen::Quaterniond(-qres.coeffs());
		this->set_orientation(qres);
		return (*this);
	}

	const CartesianPose CartesianPose::operator-(const CartesianPose& pose) const
	{
		CartesianPose result(*this);
		result -= pose;
		return result;
	}

	CartesianPose& CartesianPose::operator*=(double lambda)
	{
		// sanity check
		if(this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
		// operation
		this->set_position(lambda * this->get_position());
		// calculate the scaled rotation as a displacement from identity
		Eigen::Quaterniond w = MathTools::log(this->get_orientation());
		// calculate the orientation corresponding to the scaled velocity
		this->set_orientation(MathTools::exp(w, lambda / 2.));
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

	const CartesianPose CartesianPose::copy() const
	{
		CartesianPose result(*this);
		return result;
	}

	const Eigen::Array2d CartesianPose::dist(const CartesianPose& pose) const
	{
		// sanity check
		if(this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
		if(pose.is_empty()) throw EmptyStateException(pose.get_name() + " state is empty");
		if(!(this->get_reference_frame() == pose.get_reference_frame())) throw IncompatibleReferenceFramesException("The two states do not have the same reference frame");
		// calculation
		Eigen::Array2d result;
		// euclidean distance for position
		result(0) = (this->get_position() - pose.get_position()).norm();
		// https://math.stackexchange.com/questions/90081/quaternion-distance for orientation
		double inner_product = this->get_orientation().dot(pose.get_orientation());
		result(1) = acos(2 * inner_product * inner_product - 1);
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

	const CartesianTwist operator/(const CartesianPose& pose, const std::chrono::milliseconds& dt)
	{
		// sanity check
		if(pose.is_empty()) throw EmptyStateException(pose.get_name() + " state is empty");
		// operations
		CartesianTwist twist(pose.get_name(), pose.get_reference_frame());
		// convert the period to a double with the second as reference
		double period = std::chrono::milliseconds(dt).count();
		period /= 1000.;
		// set linear velocity
		twist.set_linear_velocity(pose.get_position() / period);
		// set angular velocity from the log of the quaternion error
		Eigen::Quaterniond log_q = MathTools::log(pose.get_orientation());
		twist.set_angular_velocity(2 * log_q.vec() / period);
		return twist;
	}

	const Eigen::Array2d dist(const CartesianPose& p1, const CartesianPose& p2)
	{
		return p1.dist(p2);
	}
}
