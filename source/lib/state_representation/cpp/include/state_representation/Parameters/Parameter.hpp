#ifndef STATEREPRESENTATION_PARAMETER_H_
#define STATEREPRESENTATION_PARAMETER_H_

#include "state_representation/State.hpp"

namespace StateRepresentation 
{
	template <typename T>
	class Parameter : public State
	{
	private:
		T value; ///< Value of the parameter

	public:
		/**
		 * @brief Empty constructor
		 */
		explicit Parameter(const std::string& name);
		
		/**
		 * @brief Constructor with a value
		 * @param value value of the parameter
		 */
		explicit Parameter(const std::string& name, const T& value);

		/**
		 * @brief Copy constructor
		 * @param parameter the parameter to copy
		 */
		Parameter(const Parameter<T>& parameter);

		/**
		 * @brief Getter of the value attribute
		 * @return the value attribute
		 */
		const T& get_value() const;

		/**
		 * @brief Getter of the value attribute
		 * @return the value attribute
		 */
		T& get_value();

		/**
		 * @brief Setter of the value attribute
		 * @param the new value attribute
		 */
		void set_value(const T& value);

		/**
	 	 * @brief Overload the ostream operator for printing
	 	 * @param os the ostream to happend the string representing the State to
	 	 * @param parameter the Parameter to print
	 	 * @return the appended ostream
	     */
		template <typename U>
		friend std::ostream& operator<<(std::ostream& os, const Parameter<U>& parameter);
	};

	template <typename T>
	Parameter<T>::Parameter(const Parameter<T>& parameter):
	State(parameter), value(parameter.value)
	{}

	template <typename T>
	inline const T& Parameter<T>::get_value() const
	{
		return this->value;
	}

	template <typename T>
	inline T& Parameter<T>::get_value()
	{
		return this->value;
	}

	template <typename T>
	inline void Parameter<T>::set_value(const T& value)
	{
		this->set_filled();
		this->value = value; 
	}

	template <typename T>
	std::ostream& operator<<(std::ostream& os, const Parameter<T>& parameter)
	{
		if(parameter.is_empty())
		{
			os << " Parameter " << parameter.get_name() << " is empty" << std::endl;
		}
		else
		{
			os << " Parameter " << parameter.get_name() << ": " << parameter.value << std::endl;
		}
  		return os;
	}
}

#endif