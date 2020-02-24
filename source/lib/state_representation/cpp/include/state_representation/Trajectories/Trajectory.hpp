#ifndef STATEREPRESENTATION_TRAJECTORIES_TRAJECTORY_H_
#define STATEREPRESENTATION_TRAJECTORIES_TRAJECTORY_H_

#include <chrono>

#include "state_representation/State.hpp"

namespace StateRepresentation
{
	template <class StateT>
	class Trajectory : public State
	{
	private:
		std::list<StateT> points;
		std::list<std::chrono::milliseconds> times;

	public:
		/**
		* @brief Empty constructor only specifying the type
		*/
		explicit Trajectory(const std::string& name, const std::string& reference="world");

		/**
		* @brief Add new point and corresponding time to trajectory
		*/
		template <typename DurationT>
		void add_point(const StateT& new_point, const std::chrono::duration<int64_t, DurationT>& new_time);

		/**
		* @brief Initialize trajectory
		*/
		void initialize();

	};
	
	template <class StateT> 
	Trajectory<StateT>::Trajectory(const std::string& robot_name, const std::string& reference):
	State(StateType::TRAJECTORY, robot_name, reference)
	{
		this->initialize();
	}

	template <class StateT>
	void Trajectory<StateT>::initialize()
	{
		this->points.clear();
		this->times.clear();
	}

	template <class StateT> template <typename DurationT>
	void Trajectory<StateT>::add_point(const StateT& new_point, const std::chrono::duration<int64_t, DurationT>& new_time)
	{
		this->points.push_back(new_point);
		auto previous_time = this->times.back();
		this->times.push_back(previous_time + new_time);
	}
}

#endif

