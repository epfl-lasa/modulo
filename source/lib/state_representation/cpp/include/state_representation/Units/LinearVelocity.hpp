#ifndef STATEREPRESENTATION_UNITS_LINEARVELOCITY_H_
#define STATEREPRESENTATION_UNITS_LINEARVELOCITY_H_

#include <chrono>
#include "state_representation/Units/Distance.hpp"

namespace StateRepresentation
{
	namespace Units
	{
		class LinearVelocity;
		
		namespace literals
		{
			/**
			 * @brief Literal operator to create a LinearVelocity in meter / second
			 * @param n the distance value in meter / second
			 * @param LinearVelocity the LinearVelocity in the base unit (meter / second)
			 */
			const LinearVelocity operator""_m_s(long double n);

			/**
			 * @brief Literal operator to create a LinearVelocity in meter / hour
			 * @param n the distance value in meter / hour
			 * @param LinearVelocity the LinearVelocity in the base unit (meter / second)
			 */
			const LinearVelocity operator""_m_h(long double n);

			/**
			 * @brief Literal operator to create a LinearVelocity in meter / millisecond
			 * @param n the distance value in meter / millisecond
			 * @param LinearVelocity the LinearVelocity in the base unit (meter / second)
			 */
			const LinearVelocity operator""_m_ms(long double n);

			/**
			 * @brief Literal operator to create a LinearVelocity in kilometer / hour
			 * @param n the distance value in kilometer / hour
			 * @param LinearVelocity the LinearVelocity in the base unit (meter / second)
			 */
			const LinearVelocity operator""_km_h(long double n);

			/**
			 * @brief Literal operator to create a LinearVelocity in kilometer / second
			 * @param n the distance value in kilometer / second
			 * @param LinearVelocity the LinearVelocity in the base unit (meter / second)
			 */
			const LinearVelocity operator""_km_s(long double n);

			/**
			 * @brief Literal operator to create a LinearVelocity in kilometer / millisecond
			 * @param n the distance value in kilometer / millisecond
			 * @param LinearVelocity the LinearVelocity in the base unit (meter / second)
			 */
			const LinearVelocity operator""_km_ms(long double n);

			/**
			 * @brief Literal operator to create a LinearVelocity in millimeter / hour
			 * @param n the distance value in millimeter / hour
			 * @param LinearVelocity the LinearVelocity in the base unit (meter / second)
			 */
			const LinearVelocity operator""_mm_h(long double n);
		}

		class LinearVelocity
		{
		private:
			long double value; ///< value of the velocity in meter / second (base unit)

		public:
			/**
			 * @brief Constructor with a value in meter / second
			 * @param n the value in meter / second
			 */
			explicit LinearVelocity(long double n=0.0);

			/**
			 * @brief Copy constructor from another LinearVelocity
			 * @param vel the LinearVelocity to copy
			 */
			LinearVelocity(const LinearVelocity& vel);

			/**
		 	 * @brief Overload the / operator for a Distance divided by a time period
		 	 * @param lhs the Distance
		 	 * @param rhs the time period
		 	 * @return the LinearVelocity as the Distance over the time period
		     */
			template <class Rep, class DurationRatio>
			friend const LinearVelocity operator/(const Distance& lhs, const std::chrono::duration<Rep, DurationRatio>& rhs);

			/**
			 * @brief Literal operator to create a LinearVelocity in meter / second
			 * @param n the distance value in meter / second
			 * @param LinearVelocity the LinearVelocity in the base unit (meter / second)
			 */
			friend const LinearVelocity literals::operator""_m_s(long double n);

			/**
			 * @brief Literal operator to create a LinearVelocity in meter / hour
			 * @param n the distance value in meter / hour
			 * @param LinearVelocity the LinearVelocity in the base unit (meter / second)
			 */
			friend const LinearVelocity literals::operator""_m_h(long double n);

			/**
			 * @brief Literal operator to create a LinearVelocity in meter / millisecond
			 * @param n the distance value in meter / millisecond
			 * @param LinearVelocity the LinearVelocity in the base unit (meter / second)
			 */
			friend const LinearVelocity literals::operator""_m_ms(long double n);

			/**
			 * @brief Literal operator to create a LinearVelocity in kilometer / hour
			 * @param n the distance value in kilometer / hour
			 * @param LinearVelocity the LinearVelocity in the base unit (meter / second)
			 */
			friend const LinearVelocity literals::operator""_km_h(long double n);

			/**
			 * @brief Literal operator to create a LinearVelocity in kilometer / second
			 * @param n the distance value in kilometer / second
			 * @param LinearVelocity the LinearVelocity in the base unit (meter / second)
			 */
			friend const LinearVelocity literals::operator""_km_s(long double n);

			/**
			 * @brief Literal operator to create a LinearVelocity in kilometer / millisecond
			 * @param n the distance value in kilometer / millisecond
			 * @param LinearVelocity the LinearVelocity in the base unit (meter / second)
			 */
			friend const LinearVelocity literals::operator""_km_ms(long double n);

			/**
			 * @brief Literal operator to create a LinearVelocity in millimeter / hour
			 * @param n the distance value in millimeter / hour
			 * @param LinearVelocity the LinearVelocity in the base unit (meter / second)
			 */
			friend const LinearVelocity literals::operator""_mm_h(long double n);
		};
	}
}

#endif