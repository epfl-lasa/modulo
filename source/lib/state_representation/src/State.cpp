#include "state_representation/State.hpp"

namespace StateRepresentation
{
	State::State(const std::string& type):
	type(type), name("none"), reference_frame("none"), empty(true)
	{}

	State::State(const std::string& type, const std::string& name, const std::string& reference_frame, const bool& empty):
	type(type), name(name), reference_frame(reference_frame), empty(empty), timestamp(std::chrono::steady_clock::now())
	{}

	State::State(const State& state):
	type(state.type), name(state.name), reference_frame(state.reference_frame), empty(state.empty), timestamp(std::chrono::steady_clock::now())
	{}

	std::ostream& operator<<(std::ostream& os, const State& state) 
	{
		if(state.is_empty())
		{
			os << "Empty State";
		}
		else
		{
			os << state.get_name() << " State expressed in " << state.get_reference_frame() << " frame" << std::endl;
	  	}
  		return os;
	}
}