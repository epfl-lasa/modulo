/**
 * @class JointVelocities
 * @brief Class to define a velocities of the joints
 * @author Baptiste Busch
 * @date 2019/09/09
 */

#ifndef STATEREPRESENTATION_JOINT_JOINTVELOCITIES_H_
#define STATEREPRESENTATION_JOINT_JOINTVELOCITIES_H_

#include "state_representation/Joint/JointState.hpp"
#include "state_representation/Joint/JointPositions.hpp"

namespace StateRepresentation 
{
	class JointPositions;
	
	class JointVelocities: public JointState
	{
	public:
		/**
		 * Empty constructor
		 */
		explicit JointVelocities();

		/**
	 	 * @brief Constructor with name and number of joints provided
	 	 * @brief name the name of the state
	 	 * @brief nb_joints the number of joints for initialization
	     */
		explicit JointVelocities(const std::string& robot_name, unsigned int nb_joints=0);

		/**
	 	 * @brief Constructor with name and list of joint names provided
	 	 * @brief name the name of the state
	 	 * @brief joint_names list of joint names
	     */
		explicit JointVelocities(const std::string& robot_name, const std::vector<std::string>& joint_names);

		/**
	 	 * @brief Constructor with name and velocities values provided
	 	 * @brief name the name of the state
	 	 * @brief velocities the vector of velocities
	     */
		explicit JointVelocities(const std::string& robot_name, const Eigen::VectorXd& velocities);

		/**
	 	 * @brief Constructor with name, a list of joint names  and velocities values provided
	 	 * @brief name the name of the state
	 	 * @brief joint_names list of joint names
	 	 * @brief velocities the vector of velocities
	     */
		explicit JointVelocities(const std::string& robot_name, const std::vector<std::string>& joint_names, const Eigen::VectorXd& velocities);

		/**
	 	 * @brief Copy constructor
	     */
		JointVelocities(const JointVelocities& velocities);

		/**
	 	 * @brief Copy constructor from a JointState
	     */
		JointVelocities(const JointState& state);

		/**
	 	 * @brief Copy constructor from a JointPositions by considering that it is equivalent to dividing the positions by 1 second
	     */
		JointVelocities(const JointPositions& positions);

		/**
	 	 * @brief Overload the += operator
	 	 * @param velocities JointVelocities to add
	 	 * @return the current JointVelocities added the JointVelocities given in argument
	     */
		JointVelocities& operator+=(const JointVelocities& velocities);

		/**
	 	 * @brief Overload the + operator
	 	 * @param velocities JointVelocities to add
	 	 * @return the current JointVelocities added the JointVelocities given in argument
	     */
		const JointVelocities operator+(const JointVelocities& velocities) const;

		/**
	 	 * @brief Overload the -= operator
	 	 * @param velocities JointVelocities to substract
	 	 * @return the current JointVelocities substracted the JointVelocities given in argument
	     */
		JointVelocities& operator-=(const JointVelocities& velocities);

		/**
	 	 * @brief Overload the - operator
	 	 * @param velocities JointVelocities to substract
	 	 * @return the current JointVelocities substracted the JointVelocities given in argument
	     */
		const JointVelocities operator-(const JointVelocities& velocities) const;

		/**
	 	 * @brief Overload the ostream operator for printing
	 	 * @param os the ostream to append the string representing the state
	 	 * @param state the state to print
	 	 * @return the appended ostream
	     */
		friend std::ostream& operator<<(std::ostream& os, const JointVelocities& velocities);

		/**
	 	 * @brief Overload the * operator with a scalar
	 	 * @param lambda the scalar to multiply with
	 	 * @return the JointVelocities provided multiply by lambda
	     */
		friend const JointVelocities operator*(double lambda, const JointVelocities& velocities);

		/**
	 	 * @brief Overload the * operator with an array of gains
	 	 * @param lambda the array to multiply with
	 	 * @return the JointVelocities provided multiply by lambda
	     */
		friend const JointVelocities operator*(const Eigen::ArrayXd& lambda, const JointVelocities& velocities);

		/**
	 	 * @brief Overload the * operator with a time period
	 	 * @param dt the time period to multiply with
	 	 * @return the JointPositions corresponding to the displacement over the time period
	     */
		friend const JointPositions operator*(const std::chrono::milliseconds& dt, const JointVelocities& velocities);

		/**
	 	 * @brief Overload the * operator with a time period
	 	 * @param dt the time period to multiply with
	 	 * @return the JointPositions corresponding to the displacement over the time period
	     */
		friend const JointPositions operator*(const JointVelocities& velocities, const std::chrono::milliseconds& dt);

		/**
	 	 * @brief Overload the / operator with a scalar
	 	 * @param lambda the scalar to divide with
	 	 * @return the JointVelocities provided divided by lambda
	     */
		friend const JointVelocities operator/(const JointVelocities& positions, double lambda);

		/**
	 	 * @brief Overload the / operator with an array of gains
	 	 * @param lambda the array to divide with
	 	 * @return the JointVelocities provided divided by lambda
	     */
		friend const JointVelocities operator/(const JointVelocities& positions, const Eigen::ArrayXd& lambda);
	};
}
#endif