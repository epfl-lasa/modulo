/**
 * @class CartesianVelocity
 * @brief Class to define CartesianVelocity in cartesian space as 3D linear_velocity and quaternion based angular_velocity
 * @author Baptiste Busch
 * @date 2019/06/07
 */

#ifndef STATEREPRESENTATION_CARTESIANVELOCITY_H_
#define STATEREPRESENTATION_CARTESIANVELOCITY_H_

#include "state_representation/Cartesian/CartesianState.hpp"
#include "state_representation/Cartesian/CartesianPose.hpp"

namespace StateRepresentation 
{
	class CartesianPose;

	class CartesianVelocity: public CartesianState
	{
	public:
		/**
	 	 * @brief Empty constructor for a CartesianVelocity. Initialize linear_velocity to zero 
	 	 * and rotation to unit quaternion.
	     */
		explicit CartesianVelocity(const std::string& name, const std::string& reference="world");

		/**
	 	 * @brief Copy constructor
	     */
		CartesianVelocity(const CartesianVelocity& v);

		/**
	 	 * @brief Copy constructor from a CartesianState
	     */
		CartesianVelocity(const CartesianState& s);

		/**
	 	 * @brief Copy constructor from a CartesianPose by considering that it is equivalent to dividing the pose by 1 second
	     */
		CartesianVelocity(const CartesianPose& p);
		
		/**
	 	 * @brief Construct a CartesianVelocity from a linear_velocity given as a vector of coordinates.
	     */
		explicit CartesianVelocity(const std::string& name, const Eigen::Vector3d&, const std::string& reference="world");

		/**
	 	 * @brief Construct a CartesianVelocity from a linear_velocity given as a vector of coordinates and a quaternion.
	     */
		explicit CartesianVelocity(const std::string& name, const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& angular_velocity, const std::string& reference="world");

		/**
	 	 * @brief Overload the += operator
	 	 * @param v CartesianVelocity to add
	 	 * @return the current CartesianVelocity added the CartesianVelocity given in argument
	     */
		CartesianVelocity& operator+=(const CartesianVelocity& v);

		/**
	 	 * @brief Overload the + operator
	 	 * @param v CartesianVelocity to add
	 	 * @return the current CartesianVelocity added the CartesianVelocity given in argument
	     */
		const CartesianVelocity operator+(const CartesianVelocity& v) const;

		/**
	 	 * @brief Overload the -= operator
	 	 * @param v CartesianVelocity to substract
	 	 * @return the current CartesianVelocity minus the CartesianVelocity given in argument
	     */
		CartesianVelocity& operator-=(const CartesianVelocity& v);

		/**
	 	 * @brief Overload the - operator
	 	 * @param v CartesianVelocity to substract
	 	 * @return the current CartesianVelocity minus the CartesianVelocity given in argument
	     */
		const CartesianVelocity operator-(const CartesianVelocity& v) const;

		/**
	 	 * @brief Overload the = operator from a CartesianState
	 	 * @param s CartesianState to get velocity from
	     */
		void operator=(const CartesianState& s);

		/**
	 	 * @brief Overload the *= operator with a scalar
	 	 * @param lambda the scalar to multiply with
	 	 * @return the CartesianVelocity multiply by lambda
	     */
		CartesianVelocity& operator*=(double lambda);

		/**
	 	 * @brief Overload the * operator with a scalar
	 	 * @param lambda the scalar to multiply with
	 	 * @return the CartesianVelocity multiply by lambda
	     */
		const CartesianVelocity operator*(double lambda) const;

		/**
	 	 * @brief Overload the ostream operator for printing
	 	 * @param os the ostream to happend the string representing the CartesianVelocity to
	 	 * @param CartesianVelocity the CartesianVelocity to print
	 	 * @return the appended ostream
	     */
		friend std::ostream& operator<<(std::ostream& os, const CartesianVelocity& velocity);

		/**
	 	 * @brief Overload the * operator with a scalar
	 	 * @param lambda the scalar to multiply with
	 	 * @return the CartesianVelocity provided multiply by lambda
	     */
		friend const CartesianVelocity operator*(double lambda, const CartesianVelocity& velocity);

		/**
	 	 * @brief Overload the * operator with a time period
	 	 * @param dt the time period to multiply with
	 	 * @return the CartesianPose corresponding to the displacement over the time period
	     */
		friend const CartesianPose operator*(const std::chrono::milliseconds& dt, const CartesianVelocity& velocity);

		/**
	 	 * @brief Overload the * operator with a time period
	 	 * @param dt the time period to multiply with
	 	 * @return the CartesianPose corresponding to the displacement over the time period
	     */
		friend const CartesianPose operator*(const CartesianVelocity& velocity, const std::chrono::milliseconds& dt);
	};
}

#endif