/**
 * @class Circular 
 * @brief Class to implement a Circular DynamicalSystem
 * @author Baptiste Busch
 * @date 2019/08/05
 *
 */

#ifndef DYNAMICAL_SYSTEMS_CIRCULAR_H_
#define DYNAMICAL_SYSTEMS_CIRCULAR_H_


#include "dynamical_systems/DynamicalSystem.hpp"
#include <cmath>

namespace DynamicalSystems
{
	template<class S>
	class Circular: public DynamicalSystem<S>
	{
	private:
		S center_;
		double radius_;
		double elevation_;

	public:
		explicit Circular(float gain=1);

		const S evaluate(const S& state) const;

		const S& get_center() const;

		void set_center(const S& center);

		double get_radius() const;

		void set_radius(double radius);

		double get_elevation() const;

		void set_elevation(double elevation);
	};

	template<class S>
	Circular<S>::Circular(float gain):
	DynamicalSystem<S>(gain),
	radius_(1), elevation_(M_PI/2)
	{}

	template<class S>
	inline const S& Circular<S>::get_center() const
	{
		return this->center_;
	}

	template<class S>
	inline void Circular<S>::set_center(const S& center)
	{
		this->center_ = center;
	}

	template<class S>
	inline double Circular<S>::get_radius() const
	{
		return this->radius_;
	}

	template<class S>
	inline void Circular<S>::set_radius(double radius)
	{
		this->radius_ = radius;
	}

	template<class S>
	inline double Circular<S>::get_elevation() const
	{
		return this->elevation_;
	}

	template<class S>
	inline void Circular<S>::set_elevation(double elevation)
	{
		this->elevation_ = elevation;
	}

	template<>
	const StateRepresentation::CartesianState Circular<StateRepresentation::CartesianState>::evaluate(const StateRepresentation::CartesianState& state) const
	{
		// StateRepresentation::CartesianPose pose_in_center = static_cast<const StateRepresentation::CartesianPose&>(this->get_center()).inverse() * static_cast<const StateRepresentation::CartesianPose&>(state);

		// double radius = pose_in_center.get_position().norm();
		// double theta = acos(pose_in_center.get_position()(2) / radius);
		// double phi = atan2(pose_in_center.get_position()(1), pose_in_center.get_position()(0));

		// double dradius = -this->get_gain() * (radius - this->get_radius());
		// double dtheta = -this->get_gain() * (theta - this->get_elevation());
		// double dphi = this->get_gain();

		// Eigen::Vector3d linear_velocity;
		// linear_velocity(0) = dradius * sin(theta) * cos(phi) + dtheta * radius * cos(theta) * cos(phi) - dphi * radius * sin(theta) * sin(phi);
		// linear_velocity(1) = dradius * sin(theta) * sin(phi) + dtheta * radius * cos(theta) * sin(phi) + dphi * radius * sin(theta) * cos(phi);
		// linear_velocity(2) = dradius * cos(theta) - dtheta * radius * sin(theta);

		// StateRepresentation::CartesianTwist velocity(state.get_name(), state.get_reference_frame());
		// velocity.set_linear_velocity(pose_in_center.inverse() * linear_velocity);

		// return velocity;

		StateRepresentation::CartesianPose pose = static_cast<const StateRepresentation::CartesianPose&>(state) - static_cast<const StateRepresentation::CartesianPose&>(this->center_);
		StateRepresentation::CartesianTwist velocity(state.get_name(), state.get_reference_frame());
		Eigen::Vector3d linear_velocity;
		linear_velocity(2) = -pose.get_position()(2);

		float R = sqrt(pose.get_position()(0) * pose.get_position()(0) + pose.get_position()(1) * pose.get_position()(1));
		float T = atan2(pose.get_position()(1), pose.get_position()(0));
		float omega = M_PI/2;

		linear_velocity(0) = -(R-this->radius_) * cos(T) - R * omega * sin(T);
		linear_velocity(1) = -(R-this->radius_) * sin(T) + R * omega * cos(T);

		velocity.set_linear_velocity(linear_velocity);
		return velocity;

	}
}
#endif