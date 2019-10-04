#include "state_representation/MathTools.hpp"

namespace StateRepresentation
{
	namespace MathTools
	{
		Eigen::Quaterniond log(const Eigen::Quaterniond & q)
		{
			Eigen::Quaterniond log_q = Eigen::Quaterniond(0,0,0,0);
			double q_norm = q.vec().norm();
			if (q_norm > 1e-4) log_q.vec() = (q.vec() / q_norm) * acos(std::min<double>(std::max<double>(q.w(),-1),1));
			return log_q;
		}

		Eigen::Quaterniond exp(const Eigen::Quaterniond & q, double lambda)
		{
			Eigen::Quaterniond exp_q = Eigen::Quaterniond::Identity();
			double q_norm = q.vec().norm();
			if (q_norm > 1e-4)
			{
				exp_q.w() = cos(q_norm * lambda);
				exp_q.vec() = (q.vec() / q_norm) * sin(q_norm * lambda);
			}
			return exp_q;
		}
	}
}