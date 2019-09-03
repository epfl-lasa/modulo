/**
 * @class Controller 
 * @brief Abstract class to define a Controller
 * @author Baptiste Busch
 * @date 2019/02/14
 *
 * A Controller represent a control algorithm, e.g. 
 * admittance control or impedance control.
 */

#ifndef MODULO_CONTROLLER_H_
#define MODULO_CONTROLLER_H_

#include "modulo_core/Cell.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "modulo_msgs/msg/jacobian_matrix.hpp"

namespace ModuloCore
{
	class Controller: public Cell 
	{
	public:
		 /**
		  * @brief Constructor for the Controller class
		  * @param node_name name of the ROS node
		  * @param period rate used by each publisher of the class
		  */
		explicit Controller(const std::string & node_name, const std::chrono::milliseconds & period, bool intra_process_comms = false);

		/**
		 * @brief Abrtract function from the lifecycle interface
		 *
		 * This function is used each time the configure call 
		 * is made from the lifecycle server. It defines all 
		 * the publishers and subscriptions of the node.
		 */
		virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State & state);
	};
}
#endif