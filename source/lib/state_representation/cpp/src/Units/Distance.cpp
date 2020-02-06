#include "state_representation/Units/Distance.hpp"

namespace StateRepresentation
{
	namespace Units
	{
		Distance::Distance(long double n):
		value(n)
		{}

		Distance::Distance(const Distance& dist):
		value(dist.value)
		{}

		Distance& Distance::operator*=(const Distance& dist)
		{
			this->value = value * dist.value;
			return (*this);
		}

		const Distance Distance::operator*(const Distance& dist) const
		{
			Distance result(*this);
			result *= dist;
			return result;
		}

		Distance& Distance::operator/=(const Distance& dist)
		{
			this->value = value / dist.value;
			return (*this);
		}

		const Distance Distance::operator/(const Distance& dist) const
		{
			Distance result(*this);
			result /= dist;
			return result;
		}

		Distance& Distance::operator+=(const Distance& dist)
		{
			this->value = value + dist.value;
			return (*this);
		}

		const Distance Distance::operator+(const Distance& dist) const
		{
			Distance result(*this);
			result += dist;
			return result;
		}

		Distance& Distance::operator-=(const Distance& dist)
		{
			this->value = value - dist.value;
			return (*this);
		}

		const Distance Distance::operator-(const Distance& dist) const
		{
			Distance result(*this);
			result -= dist;
			return result;
		}

		Distance& Distance::operator*=(double lambda)
		{
			this->value = value * lambda;
			return (*this);
		}

		const Distance Distance::operator*(double lambda) const
		{
			Distance result(*this);
			result *= lambda;
			return result;
		}

		Distance& Distance::operator/=(double lambda)
		{
			this->value = value / lambda;
			return (*this);
		}

		const Distance Distance::operator/(double lambda) const
		{
			Distance result(*this);
			result /= lambda;
			return result;
		}

		bool Distance::operator==(const Distance& rhs) const
		{
			return (abs(this->value - rhs.value) < 1e-4);
		}

		bool Distance::operator!=(const Distance& rhs) const
		{
			return !((*this) == rhs);
		}

		bool Distance::operator>(const Distance& rhs) const
		{
			return ((this->value - rhs.value) > 1e-4);
		}

		bool Distance::operator>=(const Distance& rhs) const
		{
			return (((*this) > rhs) or ((*this) == rhs));
		}

		bool Distance::operator<(const Distance& rhs) const
		{
			return ((rhs.value - this->value) > 1e-4);
		}

		bool Distance::operator<=(const Distance& rhs) const
		{
			return (((*this) < rhs) or ((*this) == rhs));
		}


		const Distance operator*(double lambda, const Distance& rhs)
		{
			return Distance(lambda * rhs.value);
		}

		namespace literals
		{
			const Distance operator""_m(long double n)
			{
				return Distance(n);
			}

			const Distance operator""_km(long double n)
			{
				return Distance(1e3 * n);
			}

			const Distance operator""_dm(long double n)
			{
				return Distance(1e-1 * n);
			}

			const Distance operator""_cm(long double n)
			{
				return Distance(1e-2 * n);
			}

			const Distance operator""_mm(long double n)
			{
				return Distance(1e-3 * n);
			}
		}
	}
}