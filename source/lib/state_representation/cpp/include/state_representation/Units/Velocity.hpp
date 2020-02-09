#ifndef STATEREPRESENTATION_UNITS_VELOCITY_H_
#define STATEREPRESENTATION_UNITS_VELOCITY_H_

#include <chrono>
#include "state_representation/Units/Velocity.hpp"

using namespace std::chrono_literals;

namespace StateRepresentation
{
	namespace Units
	{
		template <class T>
		class Velocity;

		using LinearVelocity = Velocity<Distance>;

		inline namespace literals
		{
			/**
			 * @brief Literal operator to create a Velocity in meter / second
			 * @param n the Velocity value in meter / second
			 * @param the Velocity in the base unit (meter / second)
			 */
			constexpr LinearVelocity operator""_m_s(long double n);

			/**
			 * @brief Literal operator to create a Velocity in meter / hour
			 * @param n the Velocity value in meter / hour
			 * @param the Velocity in the base unit (meter / second)
			 */
			constexpr LinearVelocity operator""_m_h(long double n);

			/**
			 * @brief Literal operator to create a Velocity in meter / millisecond
			 * @param n the Velocity value in meter / millisecond
			 * @param the Velocity in the base unit (meter / second)
			 */
			constexpr LinearVelocity operator""_m_ms(long double n);

			/**
			 * @brief Literal operator to create a Velocity in kilometer / hour
			 * @param n the Velocity value in kilometer / hour
			 * @param the Velocity in the base unit (meter / second)
			 */
			constexpr LinearVelocity operator""_km_h(long double n);

			/**
			 * @brief Literal operator to create a Velocity in kilometer / second
			 * @param n the Velocity value in kilometer / second
			 * @param the Velocity in the base unit (meter / second)
			 */
			constexpr LinearVelocity operator""_km_s(long double n);

			/**
			 * @brief Literal operator to create a Velocity in kilometer / millisecond
			 * @param n the Velocity value in kilometer / millisecond
			 * @param the Velocity in the base unit (meter / second)
			 */
			constexpr LinearVelocity operator""_km_ms(long double n);

			/**
			 * @brief Literal operator to create a Velocity in millimeter / hour
			 * @param n the Velocity value in millimeter / hour
			 * @param the Velocity in the base unit (meter / second)
			 */
			constexpr LinearVelocity operator""_mm_h(long double n);
		}

		template <class T>
		class Velocity
		{
		private:
			long double value; ///< value of the velocity in the base unit of T

		public:
			/**
			 * @brief Constructor with a value in the base unit of T
			 * @param n the value in the base unit of T
			 */
			constexpr Velocity(long double n=0.0);

			/**
			 * @brief Copy constructor from another Velocity
			 * @param vel the Velocity to copy
			 */
			constexpr Velocity(const Velocity<T>& vel);

			/**
			 * @brief Getter of the value attribute
			 * @return long double the value in meter
			 */
			constexpr long double get_value() const;

			/**
		 	 * @brief Overload the += operator
		 	 * @param vel Velocity to add
		 	 * @return the current Velocity added the Velocity given in argument
		     */
			constexpr Velocity<T>& operator+=(const Velocity<T>& vel);

			/**
		 	 * @brief Overload the + operator
		 	 * @param vel Velocity to add
		 	 * @return the current Velocity added the Velocity given in argument
		     */
			constexpr Velocity<T> operator+(const Velocity<T>& vel) const;

			/**
		 	 * @brief Overload the -= operator
		 	 * @param vel Velocity to substract
		 	 * @return the current Velocity minus the Velocity given in argument
		     */
			constexpr Velocity<T>& operator-=(const Velocity<T>& vel);

			/**
		 	 * @brief Overload the - operator
		 	 * @param vel Velocity to substract
		 	 * @return the current Velocity minus the Velocity given in argument
		     */
			constexpr Velocity<T> operator-(const Velocity<T>& vel) const;

			/**
		 	 * @brief Overload the *= operator with a scalar
		 	 * @param lambda the scalar to multiply with
		 	 * @return the Velocity multiply by lambda
		     */
			constexpr Velocity<T>& operator*=(double lambda);

			/**
		 	 * @brief Overload the * operator with a scalar
		 	 * @param lambda the scalar to multiply with
		 	 * @return the Velocity multiply by lambda
		     */
			constexpr Velocity<T> operator*(double lambda) const;

			/**
		 	 * @brief Overload the /= operator with a scalar
		 	 * @param lambda the scalar to divide by
		 	 * @return the Velocity divided by lambda
		     */
			constexpr Velocity<T>& operator/=(double lambda);

			/**
		 	 * @brief Overload the / operator with a scalar
		 	 * @param lambda the scalar to divide by
		 	 * @return the Velocity divided by lambda
		     */
			constexpr Velocity<T> operator/(double lambda) const;

			/**
			 * @brief Overload the == operator
			 * @param rhs the other Velocity to check equality with
			 * @return bool true if the two Velocitys are equal
			 */
			constexpr bool operator==(const Velocity<T>& rhs) const;

			/**
			 * @brief Overload the != operator
			 * @param rhs the other Velocity to check inequality with
			 * @return bool true if the two Velocitys are different
			 */
			constexpr bool operator!=(const Velocity<T>& rhs) const;

			/**
			 * @brief Overload the > operator
			 * @param rhs the other Velocity to check strict superiority with
			 * @return bool true if the current Velocity is strictly superior than provided Velocity
			 */
			constexpr bool operator>(const Velocity<T>& rhs) const;

			/**
			 * @brief Overload the > operator
			 * @param rhs the other Velocity to check superiority with
			 * @return bool true if the current Velocity is superior than provided Velocity
			 */
			constexpr bool operator>=(const Velocity<T>& rhs) const;

			/**
			 * @brief Overload the < operator
			 * @param rhs the other Velocity to check strict inferiority with
			 * @return bool true if the current Velocity is strictly inferior than provided Velocity
			 */
			constexpr bool operator<(const Velocity<T>& rhs) const;

			/**
			 * @brief Overload the < operator
			 * @param rhs the other Velocity to check inferiority with
			 * @return bool true if the current Velocity is inferior than provided Velocity
			 */
			constexpr bool operator<=(const Velocity<T>& rhs) const;

			/**
		 	 * @brief Overload the / operator between two Velocitys
		 	 * @param lhs the first Velocity
		 	 * @param rhs the second Velocity
		 	 * @return the ratio between first and second Velocity
		     */
			friend constexpr double operator/(const Velocity<T>& lhs, const Velocity<T>& rhs)
			{
				return lhs.value / rhs.value;
			}

			/**
		 	 * @brief Overload the * operator with a scalar on the left side
		 	 * @param lambda the scalar to multiply with
		 	 * @return the Velocity multiply by lambda
		     */
			friend constexpr Velocity<T> operator*(double lambda, const Velocity<T>& rhs)
			{
				return Velocity<T>(lambda * rhs.value);
			}

			/**
		 	 * @brief Overload the / operator for a T divided by a time period
		 	 * @param lhs the T unit
		 	 * @param rhs the time period
		 	 * @return the Velocity as the T over the time period
		     */
			template <class Rep, class DurationRatio>
			friend constexpr Velocity<T> operator/(const T& lhs, const std::chrono::duration<Rep, DurationRatio>& rhs);
		};

		template <class T>
		constexpr Velocity<T>::Velocity(long double n):
		value(n)
		{}

		template <class T>
		constexpr Velocity<T>::Velocity(const Velocity<T>& vel):
		value(vel.value)
		{}

		template <class T>
		constexpr long double Velocity<T>::get_value() const
		{
			return this->value;
		}

		template <class T>
		constexpr Velocity<T>& Velocity<T>::operator+=(const Velocity<T>& vel)
		{
			this->value = value + vel.value;
			return (*this);
		}

		template <class T>
		constexpr Velocity<T> Velocity<T>::operator+(const Velocity<T>& vel) const
		{
			Velocity<T> result(*this);
			result += vel;
			return result;
		}

		template <class T>
		constexpr Velocity<T>& Velocity<T>::operator-=(const Velocity<T>& vel)
		{
			this->value = value - vel.value;
			return (*this);
		}

		template <class T>
		constexpr Velocity<T> Velocity<T>::operator-(const Velocity<T>& vel) const
		{
			Velocity<T> result(*this);
			result -= vel;
			return result;
		}

		template <class T>
		constexpr Velocity<T>& Velocity<T>::operator*=(double lambda)
		{
			this->value = value * lambda;
			return (*this);
		}

		template <class T>
		constexpr Velocity<T> Velocity<T>::operator*(double lambda) const
		{
			Velocity<T> result(*this);
			result *= lambda;
			return result;
		}

		template <class T>
		constexpr Velocity<T>& Velocity<T>::operator/=(double lambda)
		{
			this->value = value / lambda;
			return (*this);
		}

		template <class T>
		constexpr Velocity<T> Velocity<T>::operator/(double lambda) const
		{
			Velocity<T> result(*this);
			result /= lambda;
			return result;
		}

		template <class T>
		constexpr bool Velocity<T>::operator==(const Velocity<T>& rhs) const
		{
			return (abs(this->value - rhs.value) < 1e-4);
		}

		template <class T>
		constexpr bool Velocity<T>::operator!=(const Velocity<T>& rhs) const
		{
			return !((*this) == rhs);
		}

		template <class T>
		constexpr bool Velocity<T>::operator>(const Velocity<T>& rhs) const
		{
			return ((this->value - rhs.value) > 1e-4);
		}

		template <class T>
		constexpr bool Velocity<T>::operator>=(const Velocity<T>& rhs) const
		{
			return (((*this) > rhs) or ((*this) == rhs));
		}

		template <class T>
		constexpr bool Velocity<T>::operator<(const Velocity<T>& rhs) const
		{
			return ((rhs.value - this->value) > 1e-4);
		}

		template <class T>
		constexpr bool Velocity<T>::operator<=(const Velocity<T>& rhs) const
		{
			return (((*this) < rhs) or ((*this) == rhs));
		}

		template <class T, class Rep, class DurationRatio>
		constexpr Velocity<T> operator/(const T& dist, const std::chrono::duration<Rep, DurationRatio>& rhs)
		{
			const auto rhsInSeconds = std::chrono::duration_cast<std::chrono::seconds>(rhs);
			return Velocity<T>(dist.get_value()/rhsInSeconds.count());
		}

		inline namespace literals
		{
			constexpr LinearVelocity operator""_m_s(long double n)
			{
				Distance d = 1.0_m;
				auto t = 1.0s;
				return n * (d / t);
			}

			constexpr LinearVelocity operator""_m_h(long double n)
			{
				Distance d = 1.0_m;
				auto t = 1.0h;
				return n * (d / t);
			}

			constexpr LinearVelocity operator""_m_ms(long double n)
			{
				Distance d = 1.0_m;
				auto t = 1.0ms;
				return n * (d / t);
			}

			constexpr LinearVelocity operator""_km_h(long double n)
			{
				Distance d = 1.0_km;
				auto t = 1.0h;
				return n * (d / t);
			}

			constexpr LinearVelocity operator""_km_s(long double n)
			{
				Distance d = 1.0_km;
				auto t = 1.0s;
				return n * (d / t);
			}

			constexpr LinearVelocity operator""_km_ms(long double n)
			{
				Distance d = 1.0_km;
				auto t = 1.0ms;
				return n * (d / t);
			}

			constexpr LinearVelocity operator""_mm_h(long double n)
			{
				Distance d = 1.0_mm;
				auto t = 1.0h;
				return n * (d / t);
			}
		}
	}
}

#endif