#include "dynamical_systems/Circular.hpp"

namespace DynamicalSystems
{
	Circular::Circular(const StateRepresentation::CartesianState& center, double radius, double gain, double circular_velocity):
	limit_circle_(std::make_shared<StateRepresentation::Parameter<StateRepresentation::Ellipsoid>>("limit_circle", StateRepresentation::Ellipsoid(center.get_name(), center.get_reference_frame()))),
	gain_(std::make_shared<StateRepresentation::Parameter<double>>("gain", gain)),
	circular_velocity_(std::make_shared<StateRepresentation::Parameter<double>>("circular_velocity", circular_velocity))
	{
		this->limit_circle_->get_value().set_center_state(center);
		this->limit_circle_->get_value().set_axis_lengths({radius, radius});
	}

	Circular::Circular(const StateRepresentation::Ellipsoid& limit_circle, double gain, double circular_velocity):
	limit_circle_(std::make_shared<StateRepresentation::Parameter<StateRepresentation::Ellipsoid>>(" limit_circle", limit_circle)),
	gain_(std::make_shared<StateRepresentation::Parameter<double>>("gain", gain)),
	circular_velocity_(std::make_shared<StateRepresentation::Parameter<double>>("circular_velocity", circular_velocity))
	{}

	const StateRepresentation::CartesianState Circular::evaluate(const StateRepresentation::CartesianState& state) const
	{
		// put the point in the reference of the center
		StateRepresentation::CartesianPose pose = static_cast<const StateRepresentation::CartesianPose&>(state);
		pose = this->get_center().inverse() * pose;

		StateRepresentation::CartesianTwist velocity(state.get_name(), state.get_reference_frame());
		Eigen::Vector3d linear_velocity;
		linear_velocity(2) = -this->get_gain() * pose.get_position()(2);

		float R = sqrt(pose.get_position()(0) * pose.get_position()(0) + pose.get_position()(1) * pose.get_position()(1));
		float T = atan2(pose.get_position()(1), pose.get_position()(0));
		float omega = this->get_circular_velocity();

		std::vector<double> radiuses = this->get_radiuses();
		double radius = sqrt((radiuses[0]*cos(T))*(radiuses[0]*cos(T)) + (radiuses[1]*sin(T))*(radiuses[1]*sin(T)));

		linear_velocity(0) = -this->get_gain()*(R-radius) * cos(T) - R * omega * sin(T);
		linear_velocity(1) = -this->get_gain()*(R-radius) * sin(T) + R * omega * cos(T);

		velocity.set_linear_velocity(linear_velocity);

		//compute back the linear velocity in the desired frame
		velocity = this->get_center() * velocity;
		return velocity;
	}

	const std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> Circular::get_parameters() const
	{
		std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> param_list;
		param_list.push_back(this->limit_circle_);
		param_list.push_back(this->gain_);
		param_list.push_back(this->circular_velocity_);
		return param_list;
	}
}