#include "modulo_core/Recorder.hpp"

namespace Modulo
{
	namespace Recorders
	{
		Recorder::~Recorder()
		{
			this->on_shutdown();
		}

		bool Recorder::on_configure()
		{
			return true;
		}

		bool Recorder::on_activate()
		{
			this->start_time = std::chrono::system_clock::now();
			return true;
		}

		bool Recorder::on_deactivate()
		{
			this->end_time = std::chrono::system_clock::now();
			return true;
		}

		bool Recorder::on_cleanup()
		{
			return true;
		}

		bool Recorder::on_shutdown()
		{
			return true;
		}

		void Recorder::step()
		{
			for (auto &h : this->get_handlers())
			{
				if(h.second.first->get_type() == Core::Communication::CommunicationType::SUBSCRIPTION)
				{
					const Core::Communication::MessagePassing::MessagePassingHandler& subscription = static_cast<const Core::Communication::MessagePassing::MessagePassingHandler&>(*h.second.first);
					if(!this->record(subscription.get_recipient())) RCLCPP_ERROR(this->get_logger(), "Unable to record " + subscription.get_recipient().get_name());
				}
			}
		}

		bool Recorder::record(const StateRepresentation::State& state) const
		{
			switch(state.get_type())
			{
				case StateRepresentation::StateType::CARTESIANSTATE:
					return record(static_cast<const StateRepresentation::CartesianState&>(state));

				case StateRepresentation::StateType::JOINTSTATE:
					return record(static_cast<const StateRepresentation::JointState&>(state));

				default:
					RCLCPP_ERROR(this->get_logger(), "Recording function for " + state.get_name() + " not defined for this type of state");
					return false;
			}
		}

		bool Recorder::record(const StateRepresentation::CartesianState& state) const
		{
			RCLCPP_WARN(this->get_logger(), "Trying to record " + state.get_name() + " from the base class");
			return false;
		}

		bool Recorder::record(const StateRepresentation::JointState& state) const
		{
			RCLCPP_WARN(this->get_logger(), "Trying to record " + state.get_name() + " from the base class");
			return false;
		}
	}
}