/**
 * @class CartesianTwist
 * @brief Class to define CartesianTwist in cartesian space as 3D linear_velocity and quaternion based angular_velocity
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

	class CartesianTwist: public CartesianState
	{
	public:
		/**
		 * Empty constructor
		 */
		explicit CartesianTwist();

		/**
	 	 * @brief Empty constructor for a CartesianTwist. Initialize linear_velocity to zero 
	 	 * and rotation to unit quaternion.
	     */
		explicit CartesianTwist(const std::string& name, const std::string& reference="world");

		/**
	 	 * @brief Copy constructor
	     */
		CartesianTwist(const CartesianTwist& twist);

		/**
	 	 * @brief Copy constructor from a CartesianState
	     */
		CartesianTwist(const CartesianState& state);

		/**
	 	 * @brief Copy constructor from a CartesianPose by considering that it is equivalent to dividing the pose by 1 second
	     */
		CartesianTwist(const CartesianPose& pose);
		
		/**
	 	 * @brief Construct a CartesianTwist from a linear_velocity given as a vector of coordinates.
	     */
		explicit CartesianTwist(const std::string& name, const Eigen::Vector3d&, const std::string& reference="world");

		/**
	 	 * @brief Construct a CartesianTwist from a linear_velocity given as a vector of coordinates and a quaternion.
	     */
		explicit CartesianTwist(const std::string& name, const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& angular_velocity, const std::string& reference="world");

		/**
	 	 * @brief Construct a CartesianTwist from a single 6d twist vector
	     */
		explicit CartesianTwist(const std::string& name, const Eigen::Matrix<double, 6, 1>& twist, const std::string& reference="world");

		/**
	 	 * @brief Overload the += operator
	 	 * @param twist CartesianTwist to add
	 	 * @return the current CartesianTwist added the CartesianTwist given in argument
	     */
		CartesianTwist& operator+=(const CartesianTwist& twist);

		/**
	 	 * @brief Overload the + operator
	 	 * @param twist CartesianTwist to add
	 	 * @return the current CartesianTwist added the CartesianTwist given in argument
	     */
		const CartesianTwist operator+(const CartesianTwist& twist) const;

		/**
	 	 * @brief Overload the -= operator
	 	 * @param twist CartesianTwist to substract
	 	 * @return the current CartesianTwist minus the CartesianTwist given in argument
	     */
		CartesianTwist& operator-=(const CartesianTwist& twist);

		/**
	 	 * @brief Overload the - operator
	 	 * @param twist CartesianTwist to substract
	 	 * @return the current CartesianTwist minus the CartesianTwist given in argument
	     */
		const CartesianTwist operator-(const CartesianTwist& twist) const;

		/**
	 	 * @brief Overload the = operator from a CartesianState
	 	 * @param state CartesianState to get velocity from
	     */
		void operator=(const CartesianState& state);

		/**
	 	 * @brief Overload the *= operator with a scalar
	 	 * @param lambda the scalar to multiply with
	 	 * @return the CartesianTwist multiply by lambda
	     */
		CartesianTwist& operator*=(double lambda);

		/**
	 	 * @brief Overload the * operator with a scalar
	 	 * @param lambda the scalar to multiply with
	 	 * @return the CartesianTwist multiply by lambda
	     */
		const CartesianTwist operator*(double lambda) const;

		/**
		 * @brief Clamp inplace the magnitude of the velocity to the values in argument
		 * @param max_linear the maximum magnitude of the linear velocity
		 * @param max_angular the maximum magnitude of the angular velocity
		 * @param noise_ratio if provided, this value will be used to apply a deadzone under which
		 * the velocity will be set to 0
		 */
		void clamp(double max_linear, double max_angular, double noise_ratio=0);

		/**
		 * @brief Return the clamped velocity
		 * @param max_linear the maximum magnitude of the linear velocity
		 * @param max_angular the maximum magnitude of the angular velocity
		 * @param noise_ratio if provided, this value will be used to apply a deadzone under which
		 * the velocity will be set to 0
		 * @return the clamped velocity
		 */
		const CartesianTwist clamped(double max_linear, double max_angular, double noise_ratio=0) const;

		/**
		 * @brief Return a copy of the CartesianTwist
		 * @return the copy
		 */
		const CartesianTwist copy() const;

		/**
	 	 * @brief Overload the ostream operator for printing
	 	 * @param os the ostream to happend the string representing the CartesianTwist to
	 	 * @param CartesianTwist the CartesianTwist to print
	 	 * @return the appended ostream
	     */
		friend std::ostream& operator<<(std::ostream& os, const CartesianTwist& twist);

		/**
	 	 * @brief Overload the * operator with a scalar
	 	 * @param lambda the scalar to multiply with
	 	 * @return the CartesianTwist provided multiply by lambda
	     */
		friend const CartesianTwist operator*(double lambda, const CartesianTwist& twist);

		/**
	 	 * @brief Overload the * operator with a time period
	 	 * @param dt the time period to multiply with
	 	 * @return the CartesianPose corresponding to the displacement over the time period
	     */
		friend const CartesianPose operator*(const std::chrono::milliseconds& dt, const CartesianTwist& twist);

		/**
	 	 * @brief Overload the * operator with a time period
	 	 * @param dt the time period to multiply with
	 	 * @return the CartesianPose corresponding to the displacement over the time period
	     */
		friend const CartesianPose operator*(const CartesianTwist& twist, const std::chrono::milliseconds& dt);
	};
}

#endif