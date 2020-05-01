/**
 * @author Baptiste Busch
 * @date 2019/08/05
 */

#pragma once

#include "dynamical_systems/DynamicalSystem.hpp"
#include "state_representation/Space/Cartesian/CartesianState.hpp"
#include "state_representation/Space/Cartesian/CartesianPose.hpp"
#include "state_representation/Space/Cartesian/CartesianTwist.hpp"
#include "state_representation/Robot/JointState.hpp"
#include <cmath>

namespace DynamicalSystems
{
	/**
	 * @class Circular
	 * @brief Represent a Circular dynamical system to move around an attractor
	 * @tparam S the type of space of the dynamical system (e.g. Cartesian or Joint)
	 */
	template<class S>
	class Circular: public DynamicalSystem<S>
	{
	private:
		double radius_; ///< radius of the limit circle
		double elevation_; ///< elevation of the limit circle
		double circular_velocity_; ///< velocity at wich to navigate the limit circle

	public:
		/**
		 * @brief Constructor with a provided gain 
		 * @param  gain the gain of the dynamical system (default = 1)
		 */
		explicit Circular(double gain=1);

		/**
		 * @brief Getter of the radius attribute
		 * @return the radius value
		 */
		double get_radius() const;

		/**
		 *@brief Setter of the radius attribute
		 * @param radius the new radius value
		 */
		void set_radius(double radius);

		/**
		 * @brief Getter of the evlevation attribute
		 * @return the elevation value
		 */
		double get_elevation() const;

		/**
		 *@brief Setter of the elevation attribute
		 * @param elevation the new elevation value
		 */
		void set_elevation(double elevation);

		/**
		 * @brief Getter of the circular velocity attribute
		 * @return the cirular velocity value
		 */
		double get_circular_velocity() const;

		/**
		 *@brief Setter of the circular_velocity attribute
		 * @param circular_velocity the new circular_velocity value
		 */
		void set_circular_velocity(double circular_velocity);

		/**
		 * @brief Evaluate the value of the dynamical system at a given state
		 * @param state state at wich to perform the evaluation
		 * @return the state (velocity) to move toward the attractor
		 */
		virtual const S evaluate(const S& state) const;

		/**
		 * @brief Evaluate the value of the dynamical system at a given state
		 * @param state state at wich to perform the evaluation
		 * @return the state (velocity) to move toward the attractor
		 */
		virtual const S evaluate(const std::shared_ptr<S>& state) const;
	};

	template<class S>
	Circular<S>::Circular(double gain):
	DynamicalSystem<S>(gain),
	radius_(1),
	elevation_(M_PI/2),
	circular_velocity_(M_PI/2)
	{}

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

	template<class S>
	inline double Circular<S>::get_circular_velocity() const
	{
		return this->circular_velocity_;
	}

	template<class S>
	inline void Circular<S>::set_circular_velocity(double circular_velocity)
	{
		this->circular_velocity_ = circular_velocity;
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

		StateRepresentation::CartesianPose pose = static_cast<const StateRepresentation::CartesianPose&>(state) - static_cast<const StateRepresentation::CartesianPose&>(this->get_attractor());
		StateRepresentation::CartesianTwist velocity(state.get_name(), state.get_reference_frame());
		Eigen::Vector3d linear_velocity;
		// linear_velocity(2) = -this->get_gain() * pose.get_position()(2);
		linear_velocity(2) = -this->get_gain() * pose.get_position()(2);

		float R = sqrt(pose.get_position()(0) * pose.get_position()(0) + pose.get_position()(1) * pose.get_position()(1));
		float T = atan2(pose.get_position()(1), pose.get_position()(0));
		float omega = this->circular_velocity_;

		linear_velocity(0) = -this->get_gain()*(R-this->radius_) * cos(T) - R * omega * sin(T);
		linear_velocity(1) = -this->get_gain()*(R-this->radius_) * sin(T) + R * omega * cos(T);

		velocity.set_linear_velocity(linear_velocity);
		return velocity;

	}

	template<>
	const S Circular<StateRepresentation::CartesianState>::evaluate(const std::shared_ptr<StateRepresentation::CartesianState>& state) const
	{
		return this->evaluate(*state);
	}
