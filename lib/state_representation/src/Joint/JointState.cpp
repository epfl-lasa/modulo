#include "state_representation/Joint/JointState.hpp"
#include "state_representation/Exceptions/IncompatibleStatesException.hpp"
#include "state_representation/Exceptions/EmptyStateException.hpp"

namespace StateRepresentation 
{
	JointState::JointState(const std::string& robot_name, unsigned int nb_joints):
	State("JointState", robot_name), names(nb_joints)
	{
		this->set_names(nb_joints);
	}

	JointState::JointState(const std::string& robot_name, const std::vector<std::string>& joint_names):
	State("JointState", robot_name)
	{
		this->set_names(joint_names);
	}

	JointState::JointState(const JointState& state):
	State(state), names(state.names), positions(state.positions), velocities(state.velocities),
	accelerations(state.accelerations), torques(state.torques), jacobian(state.jacobian)
	{}

	void JointState::initialize()
	{
		unsigned int size = this->names.size();
		this->set_empty(true);
		// resize
		this->positions.resize(size);
		this->velocities.resize(size);
		this->accelerations.resize(size);
		this->torques.resize(size);
		this->jacobian.resize(6, size);
		// set to zeros
		this->positions.setZero();
		this->velocities.setZero();
		this->accelerations.setZero();
		this->torques.setZero();
		this->jacobian.setZero();
	}

	JointState& JointState::operator+=(const JointState& js)
	{
		// sanity check
		if(this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
		if(js.is_empty()) throw EmptyStateException(js.get_name() + " state is empty");
		if(!this->is_compatible(js)) throw IncompatibleStatesException("The two joint states incompatible");
		// operation
		this->set_positions(this->get_positions() + js.get_positions());
		this->set_velocities(this->get_velocities() + js.get_velocities());
		this->set_accelerations(this->get_accelerations() + js.get_accelerations());
		this->set_torques(this->get_torques() + js.get_torques());
		return (*this);
	}

	const JointState JointState::operator+(const JointState& js) const
	{
		JointState result(*this);
		result += js;
		return result;
	}

	JointState& JointState::operator-=(const JointState& js)
	{
		// sanity check
		if(this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
		if(js.is_empty()) throw EmptyStateException(js.get_name() + " state is empty");
		if(!this->is_compatible(js)) throw IncompatibleStatesException("The two joint states incompatible");
		// operation
		this->set_positions(this->get_positions() - js.get_positions());
		this->set_velocities(this->get_velocities() - js.get_velocities());
		this->set_accelerations(this->get_accelerations() - js.get_accelerations());
		this->set_torques(this->get_torques() - js.get_torques());
		return (*this);
	}

	const JointState JointState::operator-(const JointState& js) const
	{
		JointState result(*this);
		result -= js;
		return result;
	}

	std::ostream& operator<<(std::ostream& os, const JointState& state) 
	{
		if(state.is_empty())
		{
			os << "empty state";
		}
		else
		{
	  		os << "names: [";
			for(auto& n:state.names) os << n << ", ";
			os << "]" << std::endl;
			os << "positions: [";
			for(unsigned int i=0; i<state.positions.size(); ++i) os << state.positions(i) << ", ";
			os << "]" << std::endl;
			os << "velocities: [";
			for(unsigned int i=0; i<state.velocities.size(); ++i) os << state.velocities(i) << ", ";
			os << "]" << std::endl;
			os << "accelerations: [";
			for(unsigned int i=0; i<state.accelerations.size(); ++i) os << state.accelerations(i) << ", ";
			os << "]" << std::endl;
			os << "torques: [";
			for(unsigned int i=0; i<state.torques.size(); ++i) os << state.torques(i) << ", ";
			os << "]" << std::endl;
			os << "jacobian:";
			for(unsigned int i=0; i<state.jacobian.rows(); ++i)
			{
				os << std::endl << "[";
				for(unsigned int j=0; j<state.jacobian.cols(); ++j) os << state.jacobian(i,j) << ", ";
				os << "]";
			}
		}
  		return os;
	}

	const JointState operator*(const float& lambda, const JointState& state)
	{
		if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty");
		JointState result(state);
		result.set_positions(lambda * state.get_positions());
		result.set_velocities(lambda * state.get_velocities());
		result.set_accelerations(lambda * state.get_accelerations());
		result.set_torques(lambda * state.get_torques());
		return result;
	}

	const JointState operator*(const Eigen::ArrayXd& lambda, const JointState& state)
	{
		if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty");
		if(lambda.size() != state.get_size()) throw IncompatibleSizeException("Gain vector is of incorrect size");
		JointState result(state);
		result.set_positions(lambda * state.get_positions().array());
		result.set_velocities(lambda * state.get_velocities().array());
		result.set_accelerations(lambda * state.get_accelerations().array());
		result.set_torques(lambda * state.get_torques().array());
		return result;
	}
}