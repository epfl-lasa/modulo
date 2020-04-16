/**
 * @class Action 
 * @brief Abstract class to define an Action for the robot to perform
 * @author Baptiste Busch
 * @date 2019/12/02
 */

#ifndef MODULO_ACTION_H_
#define MODULO_ACTION_H_

#include "modulo_core/Cell.hpp"
#include "dynamical_systems/DynamicalSystem.hpp"

namespace Modulo
{
	namespace Actions
	{
		/**
		 * @class Action
		 * @brief Abstract class to define an action to be performed by the robot
		 * @tparam S the space associated to the action (e.g Cartesian or Joint)
		 */
		template <class S>
		class Action: public Core::Cell 
		{
		private:
			std::shared_ptr<DynamicalSystems::DynamicalSystem<S> > dynamic_; //< the dynamical system associated to the action
			std::shared_ptr<S> input_state_;  //< input state of the dynamical system
			std::shared_ptr<S> output_state_; //< output state of the dynamical system

		public:
			/**
			 * @brief Constructor for the Action class
			 * @param node_name name of the ROS node
			 * @param period rate used by each publisher of the class
			 */
			template <typename DurationT>
			explicit Action(const std::shared_ptr<S>& input_state, const std::shared_ptr<S>& output_state, const std::string& node_name, const std::chrono::duration<int64_t, DurationT>& period, bool intra_process_comms=false);

			/**
			 * @brief Destructor
			 */
			~Action();

			/**
			 * @brief Getter of the dynamic as a const reference
			 * @return the dynamic attribute
			 */
			const DynamicalSystems::DynamicalSystem<S>& get_dynamic() const;

			/**
			 * @brief Getter of the dynamic as a non const reference
			 * @return the dynamic attribute
			 */
			DynamicalSystems::DynamicalSystem<S>& get_dynamic();

			/**
			 * @brief Setter of the dynamic
			 * @param dynamic the new dynamic
			 */
			void set_dynamic(const std::shared_ptr<DynamicalSystems::DynamicalSystem<S> >& dynamic);

			/**
			 * @brief Getter of the input state as a shared_ptr
			 * @return the state as a shared_ptr
			 */
			const std::shared_ptr<S>& get_input_state() const;

			/**
			 * @brief Getter of the output state as a shared_ptr
			 * @return the state as a shared_ptr
			 */
			const std::shared_ptr<S>& get_output_state() const;

			/**
			 * @brief This function is called time the configure call 
			 * is made from the lifecycle server. It is used to
			 * define behavior such as connecting to a database 
			 * or resetting an history buffer. After being 
			 * configured the node can be activated.
			 */
			virtual bool on_configure();

			/**
			 * @brief This function is called time the activate call 
			 * is made from the lifecycle server. It activates publishing
			 * and subsciptions and can be extended to start a recording
			 * or replay.
			 */
			virtual bool on_activate();

			/**
			 * @brief This function is called time the deactivate call 
			 * is made from the lifecycle server. It deactivates publishing
			 * and subsciptions and can be extended to stop a recording
			 * or a replay.
			 */
			virtual bool on_deactivate();

			/**
			 * @brief This function is called time the cleanup call 
			 * is made from the lifecycle server. It cleans the node
			 * and can be extended to close connections to a database
			 * or delete pointers. After cleanup a new configure call
			 * can be made.
			 */
			virtual bool on_cleanup();

			/**
			 * @brief This function is called time the shutdown call 
			 * is made from the lifecycle server. It terminates the node.
			 * Each elements needed to be cleaned before termination should
			 * be here.
			 */
			virtual bool on_shutdown();

			/**
			 * @brief Function computing one step of calculation. It is called periodically in the run function.
			 */
			virtual void step();
		};

		template <class S> template <typename DurationT>
		Action<S>::Action(const std::shared_ptr<S>& input_state, const std::shared_ptr<S>& output_state, const std::string& node_name, const std::chrono::duration<int64_t, DurationT>& period, bool intra_process_comms) : 
		Cell(node_name, period, intra_process_comms), input_state_(input_state), output_state_(output_state)
		{}

		template <class S>
		Action<S>::~Action()
		{
			this->on_shutdown();
		}

		template <class S>
		inline const DynamicalSystems::DynamicalSystem<S>& Action<S>::get_dynamic() const
		{
			return this->dynamic_;
		}

		template <class S>
		inline DynamicalSystems::DynamicalSystem<S>& Action<S>::get_dynamic()
		{
			return this->dynamic_;
		}

		template <class S>
		void Action<S>::set_dynamic(const std::shared_ptr<DynamicalSystems::DynamicalSystem<S> >& dynamic)
		{
			this->dynamic_ = dynamic;
		}

		template <class S>
		inline const std::shared_ptr<S>& Action<S>::get_input_state() const
		{
			return this->input_state_;
		}

		template <class S>
		inline const std::shared_ptr<S>& Action<S>::get_output_state() const
		{
			return this->output_state_;
		}

		template <class S>
		bool Action<S>::on_configure()
		{
			return true;
		}

		template <class S>
		bool Action<S>::on_activate()
		{
			return true;
		}

		template <class S>
		bool Action<S>::on_deactivate()
		{
			return true;
		}

		template <class S>
		bool Action<S>::on_cleanup()
		{
			return true;
		}

		template <class S>
		bool Action<S>::on_shutdown()
		{
			return true;
		}

		template <class S>
		void Action<S>::step()
		{
			if(!this->input_state_->is_empty())
			{
				*this->output_state_ = this->dynamic_->evaluate(this->input_state_);
			}
			else
			{
				RCLCPP_WARN(get_logger(), "state is empty initializing");
				this->output_state_->initialize();
			}
		}
	}
}
#endif