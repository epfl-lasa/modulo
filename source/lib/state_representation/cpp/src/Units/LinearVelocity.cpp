#include "state_representation/Units/LinearVelocity.hpp"

namespace StateRepresentation
{
	namespace Units
	{
		LinearVelocity::LinearVelocity(long double n):
		value(n)
		{}

		LinearVelocity::LinearVelocity(const LinearVelocity& vel):
		value(vel.value)
		{}

		template <class Rep, class DurationRatio>
		const LinearVelocity operator/(const Distance& dist, const std::chrono::duration<Rep, DurationRatio>& rhs)
		{
			const auto rhsInSeconds = std::chrono::duration_cast<std::chrono::seconds>(rhs);
			return LinearVelocity(dist.get_value()/rhsInSeconds.count());
		}

		namespace literals
		{
			const LinearVelocity operator""_m_h(long double n)
			{
				return LinearVelocity(n * 3.6e-3);
			}

			const LinearVelocity operator""_m_s(long double n)
			{
				return LinearVelocity(n);
			}

			const LinearVelocity operator""_m_ms(long double n)
			{
				return LinearVelocity(n * 1e3);
			}

			const LinearVelocity operator""_km_h(long double n)
			{
				return LinearVelocity(n * 3.6e-1);
			}

			const LinearVelocity operator""_km_s(long double n)
			{
				return LinearVelocity(n * 1e3);
			}

			const LinearVelocity operator""_km_ms(long double n)
			{
				return LinearVelocity(n * 1e6);
			}

			const LinearVelocity operator""_mm_h(long double n)
			{
				return LinearVelocity(n * 3.6e-6);
			}
		}
	}
}