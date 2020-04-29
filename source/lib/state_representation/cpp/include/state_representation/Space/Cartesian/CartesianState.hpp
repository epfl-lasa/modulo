/**
 * @class CartesianState
 * @brief Class to represent a state in Cartesian space
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef STATEREPRESENTATION_CARTESIAN_CARTESIANSTATE_H_
#define STATEREPRESENTATION_CARTESIAN_CARTESIANSTATE_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "state_representation/Space/SpatialState.hpp"

namespace StateRepresentation
{
	class CartesianState: public SpatialState
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
		 * @brief Setter of the pose from both position and orientation
		 * @param position the position
		 * @param orientation the orientation
		 */
		void set_pose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);

		/**
		 * @brief Setter of the pose from both position and orientation as Eigen 7D vector
		 * @param pose the pose
		 */
		void set_pose(const Eigen::Matrix<double, 7, 1>& pose);

		/**
		 * @brief Setter of the pose from both position and orientation as std vector
		 * @param pose the pose
		 */
		void set_pose(const std::vector<double>& pose);

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
	 	 * @brief Setter of the force and torque from a single wrench vector
	     */
		void set_wrench(const Eigen::Matrix<double, 6, 1>& wrench);

		/**
	 	 * @brief Initialize the CartesianState to a zero value
	     */
		void initialize();

		/**
		 * @brief Return a copy of the CartesianState
		 * @return the copy
		 */
		const CartesianState copy() const;

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
		 * @brief Compute the distance between two states as the sum of distances between each features
		 * @param state the second state
		 * @return dist the distance value as a double
		 */
		double dist(const CartesianState& state) const;

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

		/**
		 * @brief compute the distance between two CartesianStates
		 * @param s1 the first CartesianState
		 * @param s2 the second CartesianState
		 * @return the distance beteen the two states
		 */
		friend double dist(const CartesianState& s1, const CartesianState& s2);

		/**
		 * @brief Return the pose as a std vector of floats
		 * @return std::vector<float> the pose vector as a 7 elements vector
		 */
		virtual const std::vector<double> to_std_vector() const;
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
		twist << this->linear_velocity, this->angular_velocity;
		return twist;
	}

	inline const Eigen::Matrix<double, 6, 1> CartesianState::get_wrench() const
	{
		Eigen::Matrix<double, 6, 1> wrench;
		wrench << this->force, this->torque;
		return wrench;
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
		this->orientation = orientation.normalized();
	}

	inline void CartesianState::set_pose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation)
	{
		this->set_position(position);
		this->set_orientation(orientation);
	}

	inline void CartesianState::set_pose(const Eigen::Matrix<double, 7, 1>& pose)
	{
		this->set_filled();
		this->position = pose.head(3);
		this->orientation = Eigen::Quaterniond(pose(3), pose(4), pose(5), pose(6));
	}

	inline void CartesianState::set_pose(const std::vector<double>& pose)
	{
		this->set_filled();
		this->position = Eigen::Vector3d::Map(pose.data(), 3);
		this->orientation = Eigen::Quaterniond(pose[3], pose[4], pose[5], pose[6]);
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

	inline void CartesianState::set_wrench(const Eigen::Matrix<double, 6, 1>& wrench)
	{
		this->set_filled();
		this->force = wrench.head(3);
		this->torque = wrench.tail(3);
	}
}

#endif