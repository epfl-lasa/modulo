/**
 * @author Baptiste Busch
 * @date 2019/08/05
 */

#pragma once

#include <cmath>
#include <vector>
#include "dynamical_systems/DynamicalSystem.hpp"
#include "state_representation/Parameters/Parameter.hpp"
#include "state_representation/Space/Cartesian/CartesianState.hpp"
#include "state_representation/Space/Cartesian/CartesianPose.hpp"
#include "state_representation/Space/Cartesian/CartesianTwist.hpp"
#include "state_representation/Geometry/Ellipsoid.hpp"

namespace DynamicalSystems
{
	/**
	 * @class Circular
	 * @brief Represent a Circular dynamical system to move around an center
	 */
	class Circular: public DynamicalSystem<StateRepresentation::CartesianState>
	{
	private:
		std::shared_ptr<StateRepresentation::Parameter<StateRepresentation::Ellipsoid>> limit_circle_; ///< limit_cirlcle of the dynamical system
		std::shared_ptr<StateRepresentation::Parameter<double>> gain_; ///< gain associate to the system
		std::shared_ptr<StateRepresentation::Parameter<double>> circular_velocity_; ///< velocity at wich to navigate the limit circle

	public:
		/**
		 * @brief Default constructor with center and fixed radius
		 * @param center the center of the limit circle
		 * @param radius radius of the limit circle (default=1.)
		 * @param gain gain of the dynamical system (default=1.)
		 * @param circular_velocity circular velocity to move around the limit circle
		 */
		explicit Circular(const StateRepresentation::CartesianState& center, double radius=1.0, double gain=1.0, double circular_velocity=M_PI/2);

		/**
		 * @brief Cnstructor with an elliptic limit circle
		 * @param limit_circle the limit circle as an ellipsoid
		 * @param gain gain of the dynamical system (default=1.)
		 * @param circular_velocity circular velocity to move around the limit circle
		 */
		explicit Circular(const StateRepresentation::Ellipsoid& limit_circle, double gain=1.0, double circular_velocity=M_PI/2);

		/**
		 * @brief Getter of the center
		 * @return the center as a const reference
		 */
		const StateRepresentation::CartesianPose& get_center() const;

		/**
		 * @brief Setter of the center as a new value
		 * @param center the new center
		 */
		void set_center(const StateRepresentation::CartesianPose& center);

		/**
		 * @brief Getter of the gain attribute
		 * @return The gain value 
		 */
		double get_gain() const;

		/**
		 * @brief Getter of the gain attribute
		 * @return The gain value 
		 */
		void set_gain(double gain);

		/**
		 * @brief Getter of the radiuses of the limit circle
		 * @return the radius value
		 */
		const std::vector<double>& get_radiuses() const;

		/**
		 * @brief Setter of the radiuses of the limit circle
		 * @param radiuses the new radiuses values
		 */
		void set_radiuses(const std::vector<double>& radiuses);

		/**
		 * @brief Setter of the radius of the limit circle as a single value, i.e. perfect circle
		 * @param radiuses the new radiuses values
		 */
		void set_radius(double radius);

		/**
		 * @brief Getter of the circular velocity attribute
		 * @return the cirular velocity value
		 */
		double get_circular_velocity() const;

		/**
		 * @brief Setter of the circular_velocity attribute
		 * @param circular_velocity the new circular_velocity value
		 */
		void set_circular_velocity(double circular_velocity);

		/**
		 * @brief Getter of the limit circle attribute
		 * @return the limit circle
		 */
		const StateRepresentation::Ellipsoid& get_limit_circle() const;

		/**
		 * @brief Setter of the limit circle attribute
		 * @param the limit circle value
		 */
		void set_limit_circle(const StateRepresentation::Ellipsoid& limit_circle);

		/**
		 * @brief Evaluate the value of the dynamical system at a given state
		 * @param state state at wich to perform the evaluation
		 * @return the state (velocity) to move toward the center
		 */
		const StateRepresentation::CartesianState evaluate(const StateRepresentation::CartesianState& state) const override;

		/**
		 * @brief Return a list of all the parameters of the dynamical system
		 * @return the list of parameters
		 */
		const std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> get_parameters() const override;
	};

	inline const StateRepresentation::CartesianPose& Circular::get_center() const
	{
		return this->limit_circle_->get_value().get_center_pose();
	}

	inline void Circular::set_center(const StateRepresentation::CartesianPose& center)
	{
		this->limit_circle_->get_value().set_center_pose(center);
	}

	inline double Circular::get_gain() const
	{
		return this->gain_->get_value();
	}

	inline void Circular::set_gain(double gain)
	{
		this->gain_->set_value(gain);
	}

	inline const std::vector<double>& Circular::get_radiuses() const
	{
		return this->limit_circle_->get_value().get_axis_lengths();
	}

	inline void Circular::set_radiuses(const std::vector<double>& radiuses)
	{
		this->limit_circle_->get_value().set_axis_lengths(radiuses);
	}

	inline void Circular::set_radius(double radius)
	{
		this->limit_circle_->get_value().set_axis_lengths({radius, radius});
	}

	inline double Circular::get_circular_velocity() const
	{
		return this->circular_velocity_->get_value();
	}

	inline void Circular::set_circular_velocity(double circular_velocity)
	{
		this->circular_velocity_->set_value(circular_velocity);
	}

	inline const StateRepresentation::Ellipsoid& Circular::get_limit_circle() const
	{
		return this->limit_circle_->get_value();
	}

	inline void Circular::set_limit_circle(const StateRepresentation::Ellipsoid& limit_circle)
	{
		this->limit_circle_->set_value(limit_circle);
	}
}
