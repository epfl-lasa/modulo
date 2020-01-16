#include "state_representation/State.hpp"

namespace StateRepresentation
{
	State::State(const std::string& type):
	type(type), name("none"), empty(true)
	{}

	State::State(const std::string& type, const std::string& name, const bool& empty):
	type(type), name(name), empty(empty), timestamp(std::chrono::steady_clock::now())
	{}

	State::State(const State& state):
	type(state.type), name(state.name), empty(state.empty), timestamp(std::chrono::steady_clock::now())
	{}

	std::ostream& operator<<(std::ostream& os, const State& state) 
	{
		if(state.is_empty())
		{
			os << "Empty ";
		}
		os << " State: " << state.get_name() << std::endl;
  		return os;
	}
}