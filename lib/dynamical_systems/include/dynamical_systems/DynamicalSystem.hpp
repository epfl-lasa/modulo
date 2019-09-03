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
#include "state_representation/Cartesian/CartesianVelocity.hpp"
#include "state_representation/Joint/JointState.hpp"
#include "protocol_buffers/DynamicalSystemMsg.pb.h"

namespace DynamicalSystems
{
	template<class S>
	class DynamicalSystem 
	{
	private:
		Eigen::ArrayXd gain_;

	protected:
		ProtocolBuffers::DynamicalSystemMsg message_;

	public:
		explicit DynamicalSystem();

		explicit DynamicalSystem(const Eigen::ArrayXd& gain);

		const Eigen::ArrayXd& get_gain() const;

		void set_gain(const Eigen::ArrayXd& gain);

		virtual const S evaluate(const S& state) const;
	};

	template<class S>
	DynamicalSystem<S>::DynamicalSystem()
	{}

	template<class S>
	DynamicalSystem<S>::DynamicalSystem(const Eigen::ArrayXd& gain)
	{
		this->gain_ = gain;
		for (unsigned int i=0; i< gain.size(); ++i)
		{
			this->message_.add_gains(gain(i));
		}
	}

	template<class S>
	inline const Eigen::ArrayXd& DynamicalSystem<S>::get_gain() const
	{
		return this->gain_;
	}

	template<class S>
	inline void DynamicalSystem<S>::set_gain(const Eigen::ArrayXd& gain)
	{
		this->gain_ = gain;
		if (this->message_.gains_size() != gain.size())
		{
			this->message_.clear_gains();
			for (unsigned int i=0; i< gain.size(); ++i)
			{
				this->message_.add_gains(gain(i));
			}
		}
		else
		{
			for (unsigned int i=0; i< gain.size(); ++i)
			{
				this->message_.set_gains(i, gain(i));
			}
		}

	}

	template<class S>
	const S DynamicalSystem<S>::evaluate(const S& state) const
	{
		std::cerr << "Not implemented" << std::endl;
		return state;
	}

}
#endif