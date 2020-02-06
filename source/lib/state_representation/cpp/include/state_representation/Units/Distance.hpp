#ifndef STATEREPRESENTATION_UNITS_DISTANCE_H_
#define STATEREPRESENTATION_UNITS_DISTANCE_H_

#include <math.h>

namespace StateRepresentation
{
	namespace Units
	{
		class Distance;
		
		namespace literals
		{
			/**
			 * @brief Literal operator to create a Distance in meter
			 * @param n the distance value in meter
			 * @param Distance the Distance in the base unit (meter)
			 */
			const Distance operator""_m(long double n);

			/**
			 * @brief Literal operator to create a Distance in kilometer
			 * @param n the distance value in kilometer
			 * @param Distance the Distance in the base unit (meter)
			 */
			const Distance operator""_km(long double n);

			/**
			 * @brief Literal operator to create a Distance in decimeter
			 * @param n the distance value in decimeter
			 * @param Distance the Distance in the base unit (meter)
			 */
			const Distance operator""_dm(long double n);

			/**
			 * @brief Literal operator to create a Distance in centimeter
			 * @param n the distance value in centimeter
			 * @param Distance the Distance in the base unit (meter)
			 */
			const Distance operator""_cm(long double n);

			/**
			 * @brief Literal operator to create a Distance in millimeter
			 * @param n the distance value in millimeter
			 * @param Distance the Distance in the base unit (meter)
			 */
			const Distance operator""_mm(long double n);
		}

		class Distance
		{
		private:
			long double value; ///< value of the distance in meter (base unit)

		public:
			/**
			 * @brief Constructor with a value in meter
			 * @param n the value in meter
			 */
			explicit Distance(long double n=0.0);

			/**
			 * @brief Copy constructor from another distance
			 * @param dist the distance to copy
			 */
			Distance(const Distance& dist);

			/**
			 * @brief Getter of the value attribute
			 * @return long double the value in meter
			 */
			long double get_value() const;

			/**
		 	 * @brief Overload the *= operator
		 	 * @param dist Distance to multiply with
		 	 * @return the current Distance multiply by the Distance given in argument
		     */
			Distance& operator*=(const Distance& dist);

			/**
		 	 * @brief Overload the * operator
		 	 * @param dist Distance to multiply with
		 	 * @return the current Distance multiply by the Distance given in argument
		     */
			const Distance operator*(const Distance& dist) const;

			/**
		 	 * @brief Overload the /= operator
		 	 * @param dist Distance to divide by
		 	 * @return the current Distance divided by the Distance given in argument
		     */
			Distance& operator/=(const Distance& dist);

			/**
		 	 * @brief Overload the / operator
		 	 * @param dist Distance to divide by
		 	 * @return the current Distance divided by the Distance given in argument
		     */
			const Distance operator/(const Distance& dist) const;

			/**
		 	 * @brief Overload the += operator
		 	 * @param dist Distance to add
		 	 * @return the current Distance added the Distance given in argument
		     */
			Distance& operator+=(const Distance& dist);

			/**
		 	 * @brief Overload the + operator
		 	 * @param dist Distance to add
		 	 * @return the current Distance added the Distance given in argument
		     */
			const Distance operator+(const Distance& dist) const;

			/**
		 	 * @brief Overload the -= operator
		 	 * @param dist Distance to substract
		 	 * @return the current Distance minus the Distance given in argument
		     */
			Distance& operator-=(const Distance& dist);

			/**
		 	 * @brief Overload the - operator
		 	 * @param dist Distance to substract
		 	 * @return the current Distance minus the Distance given in argument
		     */
			const Distance operator-(const Distance& dist) const;

			/**
		 	 * @brief Overload the *= operator with a scalar
		 	 * @param lambda the scalar to multiply with
		 	 * @return the Distance multiply by lambda
		     */
			Distance& operator*=(double lambda);

			/**
		 	 * @brief Overload the * operator with a scalar
		 	 * @param lambda the scalar to multiply with
		 	 * @return the Distance multiply by lambda
		     */
			const Distance operator*(double lambda) const;

			/**
		 	 * @brief Overload the /= operator with a scalar
		 	 * @param lambda the scalar to divide by
		 	 * @return the Distance divided by lambda
		     */
			Distance& operator/=(double lambda);

			/**
		 	 * @brief Overload the / operator with a scalar
		 	 * @param lambda the scalar to divide by
		 	 * @return the Distance divided by lambda
		     */
			const Distance operator/(double lambda) const;

			/**
			 * @brief Overload the == operator
			 * @param rhs the other Distance to check equality with
			 * @return bool true if the two distances are equal
			 */
			bool operator==(const Distance& rhs) const;

			/**
			 * @brief Overload the != operator
			 * @param rhs the other Distance to check inequality with
			 * @return bool true if the two distances are different
			 */
			bool operator!=(const Distance& rhs) const;

			/**
			 * @brief Overload the > operator
			 * @param rhs the other Distance to check strict superiority with
			 * @return bool true if the current distance is strictly superior than provided distance
			 */
			bool operator>(const Distance& rhs) const;

			/**
			 * @brief Overload the > operator
			 * @param rhs the other Distance to check superiority with
			 * @return bool true if the current distance is superior than provided distance
			 */
			bool operator>=(const Distance& rhs) const;

			/**
			 * @brief Overload the < operator
			 * @param rhs the other Distance to check strict inferiority with
			 * @return bool true if the current distance is strictly inferior than provided distance
			 */
			bool operator<(const Distance& rhs) const;

			/**
			 * @brief Overload the < operator
			 * @param rhs the other Distance to check inferiority with
			 * @return bool true if the current distance is inferior than provided distance
			 */
			bool operator<=(const Distance& rhs) const;

			/**
		 	 * @brief Overload the / operator with a scalar on the left side
		 	 * @param lambda the scalar to multiply with
		 	 * @return the Distance multiply by lambda
		     */
			friend const Distance operator*(double lambda, const Distance& rhs);

			/**
			 * @brief Literal operator to create a Distance in meter
			 * @param n the distance value in meter
			 * @param Distance the Distance in the base unit (meter)
			 */
			friend const Distance literals::operator""_m(long double n);

			/**
			 * @brief Literal operator to create a Distance in kilometer
			 * @param n the distance value in kilometer
			 * @param Distance the Distance in the base unit (meter)
			 */
			friend const Distance literals::operator""_km(long double n);

			/**
			 * @brief Literal operator to create a Distance in decimeter
			 * @param n the distance value in decimeter
			 * @param Distance the Distance in the base unit (meter)
			 */
			friend const Distance literals::operator""_dm(long double n);

			/**
			 * @brief Literal operator to create a Distance in centimeter
			 * @param n the distance value in centimeter
			 * @param Distance the Distance in the base unit (meter)
			 */
			friend const Distance literals::operator""_cm(long double n);

			/**
			 * @brief Literal operator to create a Distance in millimeter
			 * @param n the distance value in millimeter
			 * @param Distance the Distance in the base unit (meter)
			 */
			friend const Distance literals::operator""_mm(long double n);
		};

		inline long double Distance::get_value() const
		{
			return this->value;
		}
	}
}

#endif