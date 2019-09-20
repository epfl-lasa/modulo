/**
 * @class DynamicalSystem
 * @brief Abstract class to define a DynamicalSystem
 * @author Baptiste Busch
 * @date 2019/07/18
 *
 */

#ifndef DYNAMICAL_SYSTEMS_DYNAMICAL_SYSTEM_H_
#define DYNAMICAL_SYSTEMS_DYNAMICAL_SYSTEM_H_


#include "state_representation/Cartesian/CartesianState.hpp"
#include "state_representation/Cartesian/CartesianPose.hpp"
#include "state_representation/Cartesian/CartesianTwist.hpp"
#include "state_representation/Joint/JointState.hpp"
#include "protocol_buffers/DynamicalSystemMsg.pb.h"
#include "dynamical_systems/Exceptions/NotImplementedException.hpp"

using namespace DynamicalSystems::Exceptions;

namespace DynamicalSystems
{
	template<class S>
	class DynamicalSystem 
	{
	private:
		double gain_;

	protected:
		ProtocolBuffers::DynamicalSystemMsg message_;

	public:
		explicit DynamicalSystem(double gain=1);

		double get_gain() const;

		void set_gain(double gain);

		virtual const S evaluate(const S& state) const;
	};

	template<class S>
	DynamicalSystem<S>::DynamicalSystem(double gain):
	gain_(gain)
	{}

	template<class S>
	inline double DynamicalSystem<S>::get_gain() const
	{
		return this->gain_;
	}

	template<class S>
	inline void DynamicalSystem<S>::set_gain(double gain)
	{
		this->gain_ = gain;
	}

	template<class S>
	const S DynamicalSystem<S>::evaluate(const S& state) const
	{
		throw NotImplementedException("This method is not implemented for abstract base class");
		return state;
	}
}
#endif