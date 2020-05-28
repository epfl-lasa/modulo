/**
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#pragma once

#include <list>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <random>
#include "state_representation/Geometry/Shape.hpp"
#include "state_representation/Space/Cartesian/CartesianPose.hpp"

namespace StateRepresentation
{
	/**
	 * @class Ellipsoid
	 */
	class Ellipsoid: public Shape 
	{
	private:
		std::vector<double> axis_lengths_; //< axis lenghts in x,y directions
		
	public:
		/**
		 * @brief Constructor with name but empty state
		 * @param name name of the ellipsoid
		 * @param safety_margin the safety_margin (default=0 in all axes)
		 */
		explicit Ellipsoid(const std::string& name, const std::string& reference_frame="world");

		/**
		 * @brief Copy constructor from another ellipsoid
		 * @param ellipsoid the ellipsoid to copy
		 */
		Ellipsoid(const Ellipsoid& ellipsoid);

		/**
		 * @brief Getter of the axis lengths
		 * @return the axis lengths
		 */
		const std::vector<double>& get_axis_lengths() const;

		/**
		 * @brief Getter of the axis length in one direction
		 * @param index the index of the length (0 for x, 1 for y and 2 for z)
		 * @return the length in the desired direction
		 */
		double get_axis_length(unsigned int index) const; 

		/**
		 * @brief Setter of the axis lengths
		 * @param axis_lengths the new values
		 */
		void set_axis_lengths(const std::vector<double>& axis_lengths);

		/**
		 * @brief Setter of the axis length at given index
		 * @param index the desired index
		 * @param axis_length the new length
		 */
		void set_axis_lengths(unsigned int index, double axis_length);

		/**
		 * @brief Function to sample an obstacle from its parameterization
		 * @param nb_samples the number of sample points to generate 
		 * @return the list of sample points
		 */
		const std::list<CartesianPose> sample_from_parameterization(unsigned int nb_samples) const;

		/**
		 * @brief Compute an ellipsoid from its algebraic equation ax2 + bxy + cy2 + cx + ey + f
		 * @return the Ellipsoid in its geometric representation
		 */
		static const Ellipsoid from_algebraic_equation(const std::string& name, const std::vector<double>& coefficients, const std::string& reference_frame="world");

		/**
		 * @brief Fit an ellipsoid on a set of points
		 * This method uses direct least square fitting from 
		 * Fitzgibbon, A., et al. (1999). "Direct least square fitting of ellipses."
 		 * IEEE Transactions on pattern analysis and machine intelligence 21(5)
		 */
		static const Ellipsoid fit(const std::string& name, const std::list<CartesianPose>& points, const std::string& reference_frame="world", double noise_level=0.01);

		/**
	 	 * @brief Overload the ostream operator for printing
	 	 * @param os the ostream to happend the string representing the Ellipsoid to
	 	 * @param ellipsoid the Ellipsoid to print
	 	 * @return the appended ostream
	     */
		friend std::ostream& operator<<(std::ostream& os, const Ellipsoid& ellipsoid);
	};

	inline const std::vector<double>& Ellipsoid::get_axis_lengths() const 
	{ 
		return this->axis_lengths_;
	}

	inline double Ellipsoid::get_axis_length(unsigned int index) const 
	{ 
		return this->axis_lengths_[index];
	}

	inline void Ellipsoid::set_axis_lengths(const std::vector<double>& axis_lengths)
	{
		this->axis_lengths_ = axis_lengths;
	}

	inline void Ellipsoid::set_axis_lengths(unsigned int index, double axis_length)
	{
		this->axis_lengths_[index] = axis_length;	
	}

}