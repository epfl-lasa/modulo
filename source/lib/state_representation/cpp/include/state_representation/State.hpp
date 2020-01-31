/**
 * @class State
 * @brief Abstract class to represent a state
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef STATEREPRESENTATION_STATE_H_
#define STATEREPRESENTATION_STATE_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <chrono>
#include <typeinfo>
#include <assert.h>
#include "state_representation/MathTools.hpp"

namespace StateRepresentation 
{
	enum class StateType
	{
		STATE,
		PARAMETER,
		CARTESIANSTATE,
		DUALQUATERNIONSTATE,
		JOINTSTATE,
		JACOBIANMATRIX

	};

	class State
	{
	private:
		StateType type; ///< type of the State
		std::string name; ///< name of the state
		bool empty;  ///< indicate if the state is empty
		std::chrono::time_point<std::chrono::steady_clock> timestamp;  ///< time since last modification made to the state
	
	public:
		/**
		 * @brief Empty constructor
		 */
		explicit State();

		/**
	 	 * @brief Constructor only specifying the type of the state from the StateType enumeration
	 	 * @param type the type of State 
	     */
		explicit State(const StateType& type);
		
		/**
	 	 * @brief Constructor with name and reference frame specification
	 	 * @param type the type of State
	 	 * @param name the name of the State
	 	 * @param empty specify if the state is initialized as empty, default true
	     */
		explicit State(const StateType& type, const std::string& name, const bool& empty=true);

		/**
	 	 * @brief Copy constructor from another State
	     */
		State(const State& state);

		/**
	 	 * @brief Getter of the type attribute
	 	 * @return the type of the State
	     */
		const StateType& get_type() const;

		/**
	 	 * @brief Getter of the empty attribute
	     */
		bool is_empty() const;

		/**
	 	 * @brief Setter of the empty attribute to true
	     */
		void set_empty();

		/**
	 	 * @brief Setter of the empty attribute to false and also reset the timestamp
	     */
		void set_filled();

		/**
	 	 * @brief Getter of the timestamp attribute
	     */
		const std::chrono::time_point<std::chrono::steady_clock>& get_timestamp() const;

		/**
	 	 * @brief Reset the timestramp attribute to now
	     */
		void reset_timestamp();

		/**
	 	 * @brief Getter of the name as const reference
	     */
		const std::string& get_name() const;

		/**
	 	 * @brief Setter of the name
	     */
		virtual void set_name(const std::string& name);

		/**
	 	 * @brief Check if the state is deprecated given a certain time delay
	 	 * @param time_delay the time after which to consider the state as deprecated
	     */
		bool is_deprecated(const std::chrono::milliseconds& time_delay);

		/**
	 	 * @brief Check if the state is compatible for operations with the state given as argument
	 	 * @param state the state to check compatibility with
	     */
		virtual bool is_compatible(const State& state) const;

		/**
	 	 * @brief Initialize the State to a zero value
	     */
		virtual void initialize();

		/**
	 	 * @brief Overload the ostream operator for printing
	 	 * @param os the ostream to happend the string representing the State to
	 	 * @param state the State to print
	 	 * @return the appended ostream
	     */
		friend std::ostream& operator<<(std::ostream& os, const State& state);
	};

	inline const StateType& State::get_type() const
	{
		return this->type;
	}

	inline bool State::is_empty() const
	{
		return this->empty;
	}

	inline void State::set_empty()
	{
		this->empty = true;
	}

	inline void State::set_filled()
	{
		this->empty = false;
		this->reset_timestamp();
	}

	inline const std::chrono::time_point<std::chrono::steady_clock>& State::get_timestamp() const
	{
		return this->timestamp;
	}

	inline void State::reset_timestamp()
	{
		this->timestamp = std::chrono::steady_clock::now();
	}

	inline bool State::is_deprecated(const std::chrono::milliseconds& time_delay)
	{
		return ((std::chrono::steady_clock::now() - this->timestamp) > time_delay);
	}

	inline const std::string& State::get_name() const
	{ 
		return this->name;
	}

	inline void State::set_name(const std::string& name)
	{
		this->name = name;
	}

	inline bool State::is_compatible(const State& state) const
	{
		bool compatible = (this->name == state.name);
		return compatible;
	}

	inline void State::initialize()
	{
		this->empty = true;
	}
}

#endif
