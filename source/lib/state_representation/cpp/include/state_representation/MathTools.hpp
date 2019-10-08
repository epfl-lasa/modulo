#ifndef STATEREPRESENTATION_MATHTOOLS_H_
#define STATEREPRESENTATION_MATHTOOLS_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace StateRepresentation
{
	namespace MathTools
	{
		/**
		 * @brief Calculate the log of a quaternion as a non-unit quaternion
		 * @param  q the quaternion to apply the log on
		 * @return the log of the quaternion
		 */
		Eigen::Quaterniond log(const Eigen::Quaterniond & q);

		/**
		 * @brief Calculate the exp of a quaternion as a non-unit quaternion
		 * @param  q the quaternion to apply the exp on
		 * @return the exp of the quaternion
		 */
		Eigen::Quaterniond exp(const Eigen::Quaterniond & q, double lambda=1);
	}
}
#endif