#include "state_representation/Cartesian/CartesianState.hpp"

namespace StateRepresentation 
{
	CartesianState::CartesianState():
	State("CartesianState")
	{
		this->initialize();
	}

	CartesianState::CartesianState(const std::string& robot_name, const std::string& reference):
	State("CartesianState", robot_name, reference)
	{
		this->initialize();
	}

	CartesianState::CartesianState(const CartesianState& state):
	State(state),
	position(state.position), orientation(state.orientation),
	linear_velocity(state.linear_velocity), angular_velocity(state.angular_velocity),
	linear_acceleration(state.linear_acceleration), angular_acceleration(state.angular_acceleration),
	force(state.force), torque(state.torque)
	{}

	void CartesianState::initialize()
	{
		this->set_empty(true);
		this->position.setZero();
		this->orientation.setIdentity();
		this->linear_velocity.setZero();
		this->angular_velocity.setZero();
		this->linear_acceleration.setZero();
		this->angular_acceleration.setZero();
		this->force.setZero();
		this->torque.setZero();
	}

	const std::string CartesianState::serialize() const
	{
		ProtocolBuffers::CartesianStateMsg message;
		message.set_name(this->get_name());
		message.set_reference_frame(this->get_reference_frame());
		for (int i=0; i<3; ++i)
		{
			message.add_position(this->position(i));
			message.add_linear_velocity(this->linear_velocity(i));
			message.add_angular_velocity(this->angular_velocity(i));
			message.add_linear_acceleration(this->linear_acceleration(i));
			message.add_angular_acceleration(this->angular_acceleration(i));
			message.add_force(this->force(i));
			message.add_torque(this->torque(i));
		}
		message.add_orientation(this->orientation.w());
	    message.add_orientation(this->orientation.x());
	    message.add_orientation(this->orientation.y());
	    message.add_orientation(this->orientation.z());
	    std::string msg_str;
		message.SerializeToString(&msg_str);
		return msg_str;
	}

	void CartesianState::deserialize(const std::string& msg_str)
	{
		ProtocolBuffers::CartesianStateMsg message;
		message.ParseFromString(msg_str);
		this->set_name(message.name());
		this->set_reference_frame(message.reference_frame());
		this->set_position(Eigen::Vector3d(message.position(0), message.position(1), message.position(2)));
		this->set_orientation(Eigen::Quaterniond(message.orientation(0), message.orientation(1), message.orientation(2), message.orientation(3)));	
		this->set_linear_velocity(Eigen::Vector3d(message.linear_velocity(0), message.linear_velocity(1), message.linear_velocity(2)));
		this->set_angular_velocity(Eigen::Vector3d(message.angular_velocity(0), message.angular_velocity(1), message.angular_velocity(2)));
		this->set_linear_acceleration(Eigen::Vector3d(message.linear_acceleration(0), message.linear_acceleration(1), message.linear_acceleration(2)));
		this->set_angular_acceleration(Eigen::Vector3d(message.angular_acceleration(0), message.angular_acceleration(1), message.angular_acceleration(2)));
		this->set_force(Eigen::Vector3d(message.force(0), message.force(1), message.force(2)));
		this->set_torque(Eigen::Vector3d(message.torque(0), message.torque(1), message.torque(2)));
	}

	std::ostream& operator<<(std::ostream& os, const CartesianState& state) 
	{ 
		if(state.is_empty())
		{
			os << "Empty CartesianState";
		}
		else
		{
			os << state.get_name() << " CartesianState expressed in " << state.get_reference_frame() << " frame" << std::endl;
	  		os << "position: (" << state.position(0) << ", ";
	  		os << state.position(1) << ", ";
	  		os << state.position(2) << ")" << std::endl;
	  		os << "orientation: (" <<state.orientation.w() << ", ";
	  		os << state.orientation.x() << ", ";
	  		os << state.orientation.y() << ", ";
	  		os << state.orientation.z() << ")";
	  		Eigen::AngleAxisd axis_angle(state.orientation);
	  		os << " <=> theta: " << axis_angle.angle() << ", ";
	  		os << "axis: (" << axis_angle.axis()(0) << ", ";
	  		os << axis_angle.axis()(1) << ", ";
	  		os << axis_angle.axis()(2) << ")" << std::endl;
	  		os << "linear velocity: (" << state.linear_velocity(0) << ", ";
	  		os << state.linear_velocity(1) << ", ";
	  		os << state.linear_velocity(2) << ")" << std::endl;
	  		os << "angular velocity: (" << state.angular_velocity(0) << ", ";
	  		os << state.angular_velocity(1) << ", ";
	  		os << state.angular_velocity(2) << ")" << std::endl;
	  		os << "linear acceleration: (" << state.linear_acceleration(0) << ", ";
	  		os << state.linear_acceleration(1) << ", ";
	  		os << state.linear_acceleration(2) << ")" << std::endl;
	  		os << "angular acceleration: (" << state.angular_acceleration(0) << ", ";
	  		os << state.angular_acceleration(1) << ", ";
	  		os << state.angular_acceleration(2) << ")" << std::endl;
	  		os << "force: (" << state.force(0) << ", ";
	  		os << state.force(1) << ", ";
	  		os << state.force(2) << ")" << std::endl;
	  		os << "torque: (" << state.torque(0) << ", ";
	  		os << state.torque(1) << ", ";
	  		os << state.torque(2) << ")";
	  	}
  		return os;
	}
}