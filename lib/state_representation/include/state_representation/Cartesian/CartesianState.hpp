/**
 * @class CartesianState
 * @brief Class to represent a state in Cartesian space
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef STATEREPRESENTATION_CARTESIANSTATE_H_
#define STATEREPRESENTATION_CARTESIANSTATE_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "state_representation/State.hpp"
#include "protocol_buffers/CartesianStateMsg.pb.h"

namespace StateRepresentation 
{
	class CartesianState: public State
	{
	private:
		Eigen::Vector3d position; ///< position of the point
		Eigen::Quaterniond orientation; ///< orientation of the point
		Eigen::Vector3d linear_velocity; ///< linear_velocity of the point
		Eigen::Vector3d angular_velocity; ///< angular_velocity of the point
		Eigen::Vector3d linear_acceleration; ///< linear_acceleration of the point
		Eigen::Vector3d angular_acceleration; ///< angular_acceleration of the point
		Eigen::Vector3d force; ///< force applied at the point
		Eigen::Vector3d torque; ///< torque applied at the point

	public:
		/**
	 	 * @brief Empty constructor
	     */
		explicit CartesianState();
		
		/**
	 	 * @brief Constructor with name and reference frame provided
	 	 * @brief name the name of the state
	 	 * @brief reference the name of the reference frame
	     */
		explicit CartesianState(const std::string& name, const std::string& reference="world");

		/**
	 	 * @brief Copy constructor of a CartesianState
	     */
		CartesianState(const CartesianState& state);

		/**
	 	 * @brief Getter of the posistion attribute
	     */
		const Eigen::Vector3d& get_position() const;

		/**
	 	 * @brief Getter of the orientation attribute
	     */
		const Eigen::Quaterniond& get_orientation() const;

		/**
	 	 * @brief Getter of the linear velocity attribute 
	     */
		const Eigen::Vector3d& get_linear_velocity() const;

		/**
	 	 * @brief Getter of the angular velocity attribute 
	     */
		const Eigen::Vector3d& get_angular_velocity() const;

		/**
	 	 * @brief Getter of the linear acceleration attribute 
	     */
		const Eigen::Vector3d& get_linear_acceleration() const;

		/**
	 	 * @brief Getter of the angular acceleration attribute 
	     */
		const Eigen::Vector3d& get_angular_acceleration() const;

		/**
	 	 * @brief Getter of the force attribute
	     */
		const Eigen::Vector3d& get_force() const;

		/**
	 	 * @brief Getter of the torque attribute
	     */
		const Eigen::Vector3d& get_torque() const;

		/**
	 	 * @brief Getter of a pose from position and orientation attributes
	 	 * @return the pose as a 4x4 transformation matrix
	     */
		const Eigen::Matrix4d get_pose() const;

		/**
	 	 * @brief Getter of a twist from velocities attribute
	     */
		const Eigen::Matrix<double, 6, 1> get_twist() const;

		/**
	 	 * @brief Getter of a wrench from force and torque attributes
	     */
		const Eigen::Matrix<double, 6, 1> get_wrench() const;

		/**
	 	 * @brief Setter of the name
	     */
		void set_name(const std::string& name);

		/**
	 	 * @brief Setter of the reference frame
	     */
		void set_reference_frame(const std::string& reference);

		/**
	 	 * @brief Setter of the position
	     */
		void set_position(const Eigen::Vector3d& position);

		/**
	 	 * @brief Setter of the position from three scalar coordinates
	     */
		void set_position(const double& x, const double& y, const double& z);

		/**
	 	 * @brief Setter of the orientation
	     */
		void set_orientation(const Eigen::Quaterniond& orientation);

		/**
	 	 * @brief Setter of the linear velocity attribute
	     */
		void set_linear_velocity(const Eigen::Vector3d& linear_velocity);

		/**
	 	 * @brief Setter of the angular velocity attribute
	     */
		void set_angular_velocity(const Eigen::Vector3d& angular_velocity);

		/**
	 	 * @brief Setter of the linear accelration attribute
	     */
		void set_linear_acceleration(const Eigen::Vector3d& linear_acceleration);

		/**
	 	 * @brief Setter of the angular velocity attribute
	     */
		void set_angular_acceleration(const Eigen::Vector3d& angular_acceleration);

		/**
	 	 * @brief Setter of the force attribute
	     */
		void set_force(const Eigen::Vector3d& force);

		/**
	 	 * @brief Setter of the force attribute
	     */
		void set_torque(const Eigen::Vector3d& torque);

		/**
	 	 * @brief Setter of the linear and angular velocities from a single twist vector
	     */
		void set_twist(const Eigen::Matrix<double, 6, 1>& twist);

		/**
	 	 * @brief Initialize the CartesianState to a zero value
	     */
		void initialize();

		/**
	 	 * @brief Serialize the state to a string using protobuf
	 	 * @return the serialized object
	     */
		const std::string serialize() const;

		/**
	 	 * @brief Deserialize the object from a string
	 	 * @param msg_str the serialized string
	     */
		void deserialize(const std::string& msg_str);

		/**
	 	 * @brief Overload the *= operator with a scalar
	 	 * @param lambda the scalar to multiply with
	 	 * @return the CartesianState multiply by lambda
	     */
		CartesianState& operator*=(double lambda);

		/**
	 	 * @brief Overload the * operator with a scalar
	 	 * @param lambda the scalar to multiply with
	 	 * @return the CartesianState multiply by lambda
	     */
		const CartesianState operator*(double lambda) const;

		/**
	 	 * @brief Overload the ostream operator for printing
	 	 * @param os the ostream to happend the string representing the state to
	 	 * @param state the state to print
	 	 * @return the appended ostream
	     */
		friend std::ostream& operator<<(std::ostream& os, const CartesianState& state);
		
		/**
	 	 * @brief Overload the * operator with a scalar
	 	 * @param lambda the scalar to multiply with
	 	 * @return the CartesianState provided multiply by lambda
	     */
		friend const CartesianState operator*(double lambda, const CartesianState& state);
	};

	inline const Eigen::Vector3d& CartesianState::get_position() const
	{ 
		return this->position;
	}

	inline const Eigen::Quaterniond& CartesianState::get_orientation() const
	{ 
		return this->orientation;
	}

	inline const Eigen::Vector3d& CartesianState::get_linear_velocity() const
	{
		return this->linear_velocity;
	}

	inline const Eigen::Vector3d& CartesianState::get_angular_velocity() const
	{
		return this->angular_velocity;
	}

	inline const Eigen::Vector3d& CartesianState::get_linear_acceleration() const
	{
		return this->linear_acceleration;
	}

	inline const Eigen::Vector3d& CartesianState::get_angular_acceleration() const
	{
		return this->angular_acceleration;
	}

	inline const Eigen::Vector3d& CartesianState::get_force() const
	{
		return this->force;
	}

	inline const Eigen::Vector3d& CartesianState::get_torque() const
	{
		return this->torque;
	}

	inline const Eigen::Matrix4d CartesianState::get_pose() const
	{
		Eigen::Matrix4d pose;
		pose << this->orientation.toRotationMatrix(), this->position, 0., 0., 0., 1;
		return pose;
	}

	inline const Eigen::Matrix<double, 6, 1> CartesianState::get_twist() const
	{
		Eigen::Matrix<double, 6, 1> twist;
		twist << this->linear_velocity, this->linear_acceleration;
		return twist;
	}

	inline const Eigen::Matrix<double, 6, 1> CartesianState::get_wrench() const
	{
		Eigen::Matrix<double, 6, 1> wrench;
		wrench << this->force, this->torque;
		return wrench;
	}

	inline void CartesianState::set_name(const std::string& name)
	{
		this->State::set_name(name);
	}

	inline void CartesianState::set_reference_frame(const std::string& reference)
	{
		this->State::set_reference_frame(reference);
	}

	inline void CartesianState::set_position(const Eigen::Vector3d& position)
	{
		this->set_filled();
		this->position = position;
	}

	inline void CartesianState::set_position(const double& x, const double& y, const double& z)
	{
		this->set_filled();
		this->set_position(Eigen::Vector3d(x, y, z));
	}

	inline void CartesianState::set_orientation(const Eigen::Quaterniond& orientation)
	{
		this->set_filled();
		this->orientation = (this->orientation.dot(orientation)) > 0 ? orientation.normalized() : Eigen::Quaterniond(-(orientation.normalized()).coeffs());
	}

	inline void CartesianState::set_linear_velocity(const Eigen::Vector3d& linear_velocity)
	{
		this->set_filled();
		this->linear_velocity = linear_velocity;
	}

	inline void CartesianState::set_angular_velocity(const Eigen::Vector3d& angular_velocity)
	{
		this->set_filled();
		this->angular_velocity = angular_velocity;
	}

	inline void CartesianState::set_linear_acceleration(const Eigen::Vector3d& linear_acceleration)
	{
		this->set_filled();
		this->linear_acceleration = linear_acceleration;
	}

	inline void CartesianState::set_angular_acceleration(const Eigen::Vector3d& angular_acceleration)
	{
		this->set_filled();
		this->angular_acceleration = angular_acceleration;
	}

	inline void CartesianState::set_force(const Eigen::Vector3d& force)
	{
		this->set_filled();
		this->force = force;
	}

	inline void CartesianState::set_torque(const Eigen::Vector3d& torque)
	{
		this->set_filled();
		this->torque = torque;
	}

	inline void CartesianState::set_twist(const Eigen::Matrix<double, 6, 1>& twist)
	{
		this->set_filled();
		this->linear_velocity = twist.head(3);
		this->angular_velocity = twist.tail(3);
	}
}

#endif