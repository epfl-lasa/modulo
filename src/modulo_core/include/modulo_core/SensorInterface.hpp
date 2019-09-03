/**
 * @class Sensor
 * @brief Abstract class to define a Sensor
 * @author Baptiste Busch
 * @date 2019/02/14
 *
 * A SensorInterface output values observed by a physical sensor.
 */

#ifndef MODULO_SENSOR_INTERFACE_H_
#define MODULO_SENSOR_INTERFACE_H_

#include "modulo_core/Cell.hpp"

namespace ModuloCore
{
	class SensorInterface: public Cell 
	{
	public:
		/**
		 * @brief Constructor for the SensorInterface class
		 * @param node_name name of the ROS node
		 * @param period rate used by each publisher of the class
		 */
		explicit SensorInterface(const std::string & node_name, const std::chrono::milliseconds & period, bool intra_process_comms = false);

		/** 
		 * @brief Abrtract function from the lifecycle interface
		 *
		 * This function is used each time the configure call 
		 * is made from the lifecycle server. It defines all 
		 * the publishers and subscriptions of the node.
		 */
		virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State & state);

		/**
		 * @brief Function computing one step of calculation. It is called periodically in the run function.
		 */
		virtual void step()=0;
	};
}
#endif