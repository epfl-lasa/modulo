#ifndef STATEREPRESENTATION_TRAJECTORIES_TRAJECTORY_H_
#define STATEREPRESENTATION_TRAJECTORIES_TRAJECTORY_H_

#include <chrono>
#include <deque>
#include "state_representation/State.hpp"

namespace StateRepresentation
{
	template <class StateT>
	class Trajectory : public State
	{
	private:
		std::deque<StateT> points;
		std::deque<std::chrono::milliseconds> times;

	public:

		/**
	 	 * @brief Empty constructor
	     */
		explicit Trajectory();

		/**
	 	 * @brief Constructor with name and reference frame provided
	 	 * @brief name the name of the state
	 	 * @brief reference the name of the reference frame
	     */
		explicit Trajectory(const std::string& name, const std::string& reference="world");

		/**
		* @brief Initialize trajectory
		*/
		void initialize();

		/**
		* @brief Add new point and corresponding time to trajectory
		*/
		template <typename DurationT>
		void add_point(const StateT& new_point, const std::chrono::duration<int64_t, DurationT>& new_time);

		/**
		* @brief Insert new point and corresponding time to trajectory between two already existing points
		*/
		template <typename DurationT>
		void insert_point(const StateT& new_point, const std::chrono::duration<int64_t, DurationT>& new_time, int pos);

		/**
		* @brief Delete last point and corresponding time from trajectory
		*/
		void delete_point();

		/**
		* @brief Clear trajectory
		*/
		void clear();

		/**
		* @brief Get attribute list of trajectory points
		*/
		const std::deque<StateT>& get_points() const;

		/**
		* @brief Get attribute list of trajectory times
		*/
		const std::deque<std::chrono::milliseconds>& get_times() const;

		/**
		* @brief Operator overload for returning a single trajectory point and corresponding time
		*/
		const std::pair<StateT, std::chrono::milliseconds> operator[](unsigned int idx) const;

		/**
		* @brief Operator overload for returning a single trajectory point and corresponding time
		*/
		std::pair<StateT, std::chrono::milliseconds> operator[](unsigned int idx);

	};
	
	template <class StateT> 
	Trajectory<StateT>::Trajectory():
	State(StateType::TRAJECTORY)
	{
		this->initialize();
	}

	template <class StateT> 
	Trajectory<StateT>::Trajectory(const std::string& name, const std::string& reference):
	State(StateType::TRAJECTORY, name, reference)
	{
		this->initialize();
	}

	template <class StateT>
	void Trajectory<StateT>::initialize()
	{
		this->State::initialize();
		this->points.clear();
		this->times.clear();
	}

	template <class StateT> template <typename DurationT>
	void Trajectory<StateT>::add_point(const StateT& new_point, const std::chrono::duration<int64_t, DurationT>& new_time)
	{
		this->set_filled();
		this->points.push_back(new_point);
		if(!this->times.empty())
		{
			auto previous_time = this->times.back();
			this->times.push_back(previous_time + new_time);
		}
		else
			this->times.push_back(new_time);
	}

	template <class StateT> template <typename DurationT>
	void Trajectory<StateT>::insert_point(const StateT& new_point, const std::chrono::duration<int64_t, DurationT>& new_time, int pos)
	{
		this->set_filled();

		auto it_points = this->points.begin();
		auto it_times = this->times.begin();
		std::advance(it_points, pos);
		std::advance(it_times, pos);
		
		this->points.insert(it_points, new_point);

		auto previous_time = this->times[pos-1];
		this->times.insert(it_times, previous_time + new_time);

		for(unsigned int i = pos+1; i <= points.size(); i++)
			this->times[i] = times[i] + new_time;
	}

	template <class StateT>
	void Trajectory<StateT>::delete_point()
	{
		this->set_filled();
		if(!this->points.empty())
			this->points.pop_back();
		if(!this->times.empty())
			this->times.pop_back();
	}

	template <class StateT>
	void Trajectory<StateT>::clear()
	{
		this->points.clear();
		this->times.clear();
	}

	template <class StateT> 
	inline const std::deque<StateT>& Trajectory<StateT>::get_points() const
	{
		return this->points;
	}

	template <class StateT> 
	inline const std::deque<std::chrono::milliseconds>& Trajectory<StateT>::get_times() const 
	{
		return this->times;
	}

	template <class StateT>
	const std::pair<StateT, std::chrono::milliseconds> Trajectory<StateT>::operator[](unsigned int idx) const
	{
		return std::make_pair(this->points[idx], this->times[idx]);
	}

	template <class StateT>
	std::pair<StateT, std::chrono::milliseconds> Trajectory<StateT>::operator[](unsigned int idx)
	{
		this->set_filled();
		return std::make_pair(this->points[idx], this->times[idx]);
	}
}

#endif
