/**
 * @class CartesianPose
 * @brief Class to define CartesianPose in cartesian space as 3D position and quaternion based orientation
 * @author Baptiste Busch
 * @date 2019/06/07
 */

#ifndef STATEREPRESENTATION_CARTESIANPOSE_H_
#define STATEREPRESENTATION_CARTESIANPOSE_H_

#include "state_representation/Cartesian/CartesianState.hpp"
#include "state_representation/Cartesian/CartesianVelocity.hpp"

namespace StateRepresentation 
{
	class CartesianVelocity;
	
	class CartesianPose: public CartesianState
	{
	public:
		explicit CartesianPose();

		/**
	 	 * @brief Constructor with name and reference frame provided
	 	 * @brief name the name of the state
	 	 * @brief reference the name of the reference frame
	     */
		explicit CartesianPose(const std::string& name, const std::string& reference="world");

		/**
	 	 * @brief Copy constructor.
	     */
		CartesianPose(const CartesianPose& p);

		/**
	 	 * @brief Copy constructor from a CartesianState.
	     */
		CartesianPose(const CartesianState& s);
		
		/**
	 	 * @brief Construct a CartesianPose from a position given as a vector of coordinates.
	     */
		explicit CartesianPose(const std::string& name, const Eigen::Vector3d& position, const std::string& reference="world");

		/**
	 	 * @brief Construct a CartesianPose from a position given as three scalar coordinates.
	     */
		explicit CartesianPose(const std::string& name, const double& x, const double& y, const double& z, const std::string& reference="world");

		/**
	 	 * @brief Construct a CartesianPose from a position given as a vector of coordinates and a quaternion.
	     */
		explicit CartesianPose(const std::string& name, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, const std::string& reference="world");

		/**
	 	 * @brief Overload the *= operator
	 	 * @param p CartesianPose to multiply with
	 	 * @return the current CartesianPose multiply by the CartesianPose given in argument
	     */
		CartesianPose& operator*=(const CartesianPose& p);

		/**
	 	 * @brief Overload the * operator
	 	 * @param p CartesianPose to multiply with
	 	 * @return the current CartesianPose multiply by the CartesianPose given in argument
	     */
		const CartesianPose operator*(const CartesianPose& p) const;

		/**
	 	 * @brief Overload the *= operator with a CartesianState
	 	 * @param p CartesianState to multiply with
	 	 * @return the current CartesianPose multiply by the CartesianState given in argument
	     */
		const CartesianState operator*(const CartesianState& s) const;

		/**
	 	 * @brief Overload the * operator for a vector input
	 	 * @param v vector to multiply with, representing either a position, velocity or acceleration
	 	 * @return the vector multiplied by the current CartesianPose
	     */
		const Eigen::Vector3d operator*(const Eigen::Vector3d& v) const;

		/**
	 	 * @brief Overload the += operator
	 	 * @param p CartesianPose to add
	 	 * @return the current CartesianPose added the CartesianPose given in argument
	     */
		CartesianPose& operator+=(const CartesianPose& p);

		/**
	 	 * @brief Overload the + operator
	 	 * @param p CartesianPose to add
	 	 * @return the current CartesianPose added the CartesianPose given in argument
	     */
		const CartesianPose operator+(const CartesianPose& p) const;

		/**
	 	 * @brief Overload the += operator with a CartesianVelocity
	 	 * @param v CartesianVelocity to add
	 	 * @return the current CartesianPose added the CartesianVelocity given in argument
	     */
		//CartesianPose& operator+=(const CartesianVelocity& v);

		/**
	 	 * @brief Overload the + operator with a CartesianVelocity
	 	 * @param v CartesianVelocity to add
	 	 * @return the current CartesianPose added the CartesianVelocity given in argument
	     */
		//const CartesianPose operator+(const CartesianVelocity& v) const;

		/**
	 	 * @brief Overload the -= operator
	 	 * @param p CartesianPose to substract
	 	 * @return the current CartesianPose minus the CartesianPose given in argument
	     */
		CartesianPose& operator-=(const CartesianPose& p);

		/**
	 	 * @brief Overload the - operator
	 	 * @param p CartesianPose to substract
	 	 * @return the current CartesianPose minus the CartesianPose given in argument
	     */
		const CartesianPose operator-(const CartesianPose& p) const;

		/**
	 	 * @brief Overload the *= operator with a scalar
	 	 * @param lambda the scalar to multiply with
	 	 * @return the CartesianPose multiply by lambda
	     */
		CartesianPose& operator*=(double lambda);

		/**
	 	 * @brief Overload the * operator with a scalar
	 	 * @param lambda the scalar to multiply with
	 	 * @return the CartesianPose multiply by lambda
	     */
		const CartesianPose operator*(double lambda) const;

		/**
		 * @brief compute the inverse of the current CartesianPose
		 * @return the inverse
		 */
		const CartesianPose inverse() const;

		/**
	 	 * @brief Overload the ostream operator for printing
	 	 * @param os the ostream to happend the string representing the CartesianPose to
	 	 * @param CartesianPose the CartesianPose to print
	 	 * @return the appended ostream
	     */
		friend std::ostream& operator<<(std::ostream& os, const CartesianPose& pose);

		/**
	 	 * @brief Overload the * operator with a scalar
	 	 * @param lambda the scalar to multiply with
	 	 * @return the CartesianPose provided multiply by lambda
	     */
		friend const CartesianPose operator*(double lambda, const CartesianPose& pose);

		/**
	 	 * @brief Overload the / operator with a time period
	 	 * @param dt the time period to divise by
	 	 * @return the corresponding CartesianVelocity
	     */
		friend const CartesianVelocity operator/(const CartesianPose& pose, const std::chrono::milliseconds& dt);
	};
}

#endif