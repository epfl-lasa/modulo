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

namespace StateRepresentation 
{
	class State
	{
	private:
		std::string type; ///< type of the State (Cartesian, DualQuaternion or Joint)
		std::string name; ///< name of the frame
		std::string reference_frame; ///< name of the reference frame
		bool empty;  ///< indicate if the state is empty
		std::chrono::time_point<std::chrono::steady_clock> timestamp;  ///< time since last modification made to the state
	
	public:
		/**
	 	 * @brief Empty constructor only specifying the type
	     */
		explicit State(const std::string& type);
		
		/**
	 	 * @brief Constructor with name and reference frame specification
	 	 * @param type the type of State (Cartesian, DualQuaternion or Joint)
	 	 * @param name the name of the State
	 	 * @param reference_frame the reference frame in which the state is expressed, by default world
	 	 * @param empty specify if the state is initialized as empty, default true
	     */
		explicit State(const std::string& type, const std::string& name, const std::string& reference_frame="world", const bool& empty=true);

		/**
	 	 * @brief Copy constructor from another State
	     */
		State(const State& state);

		/**
	 	 * @brief Getter of the type attribute
	     */
		const std::string& get_type() const;

		/**
	 	 * @brief Getter of the type attribute
	     */
		std::string& get_type();

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
	 	 * @brief Getter of the reference frame as const reference
	     */
		const std::string get_reference_frame() const;

		/**
	 	 * @brief Getter of the name as const reference
	     */
		const std::string& get_name() const;

		/**
	 	 * @brief Setter of the reference frame
	     */
		virtual void set_reference_frame(const std::string& reference);

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

	inline const std::string& State::get_type() const
	{
o		return this->type;
	}

	inline std::string& State::get_type()
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

	inline const std::string State::get_reference_frame() const
	{ 
		return this->reference_frame;
	}

	inline const std::string& State::get_name() const
	{ 
		return this->name;
	}

	inline void State::set_reference_frame(const std::string& reference)
	{
		this->reference_frame = reference;
	}

	inline void State::set_name(const std::string& name)
	{
		this->name = name;
	}

	inline bool State::is_compatible(const State& state) const
	{
		bool compatible = (this->name == state.name) && (this->reference_frame == state.reference_frame);
		return compatible;
	}

	inline void State::initialize()
	{
		this->empty = true;
	}
}

#endif
