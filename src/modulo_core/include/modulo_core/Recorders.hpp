/**
 * @class Recorder
 * @brief Abstract class to define a Recorder
 * @author Baptiste Busch
 * @date 2019/06/13
 *
 * A Recorder is used to listen on specific topics and record
 * the received data.
 */

#ifndef MODULORECORDER_RECORDER_H_
#define MODULORECORDER_RECORDER_H_

#include "modulo_core/Cell.hpp"
#include <typeinfo>

namespace Modulo
{
	namespace Recorders
	{
		class Recorder: public Core::Cell 
		{
		private:
			std::chrono::time_point<std::chrono::system_clock> start_time;
			std::chrono::time_point<std::chrono::system_clock> end_time;

		public:
			/**
			 * @brief Constructor for the Recorder class
			 * @param node_name name of the ROS node
			 * @param period rate used by each publisher of the class
			 */
			explicit Recorder(const std::string & node_name, const std::chrono::milliseconds & period, bool intra_process_comms = false);

			inline const auto& get_start_time() const
			{
				return start_time;
			}

			inline const auto& get_end_time() const
			{
				return end_time;
			}

			/**
			 * @brief This function is called time the configure call 
			 * is made from the lifecycle server. It is used to
			 * define behavior such as connecting to a database 
			 * or resetting an history buffer. After being 
			 * configured the node can be activated.
			 */
			virtual void on_configure();

			/**
			 * @brief This function is called time the activate call 
			 * is made from the lifecycle server. It activates publishing
			 * and subsciptions and can be extended to start a recording
			 * or replay.
			 */
			virtual void on_activate();

			/**
			 * @brief This function is called time the deactivate call 
			 * is made from the lifecycle server. It deactivates publishing
			 * and subsciptions and can be extended to stop a recording
			 * or a replay.
			 */
			virtual void on_deactivate();

			/**
			 * @brief This function is called time the cleanup call 
			 * is made from the lifecycle server. It cleans the node
			 * and can be extended to close connections to a database
			 * or delete pointers. After cleanup a new configure call
			 * can be made.
			 */
			virtual void on_cleanup();

			/**
			 * @brief This function is called time the shutdown call 
			 * is made from the lifecycle server. It terminates the node.
			 * Each elements needed to be cleaned before termination should
			 * be here.
			 */
			virtual void on_shutdown();

			/**
			 * @brief Function computing one step of calculation. It is called periodically in the run function.
			 */
			virtual void step() = 0;

			bool record(const StateRepresentation::State& state) const;

			virtual bool record(const StateRepresentation::CartesianState& state) const=0;
		};
	}
}
#endif