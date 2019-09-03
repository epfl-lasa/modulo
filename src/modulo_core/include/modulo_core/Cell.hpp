/**
 * @class Cell
 * @brief Abstract class to define a Call
 * @author Baptiste Busch
 * @date 2019/02/14
 *
 * A Cell is the base class of the whole architecture.
 * It handles all the basic ROS communications such as
 * definitions of subrscriptions, publishers and service
 * calls. It can then be derived into a MotionGenerator,
 * a Controller, a Sensor, or a RobotInterface. It is
 * derived from a lifecyle node which allows to use
 * ROS2 state machine functionnalities for nodes.
 */

#ifndef MODULO_CELL_H_
#define MODULO_CELL_H_

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp/function_traits.hpp"
#include "rcutils/logging_macros.h"
#include "modulo_core/Communication/SubscriptionHandler.hpp"
#include "modulo_core/Communication/PublisherHandler.hpp"
#include "modulo_core/Communication/TransformBroadcasterHandler.hpp"
#include "modulo_core/Communication/TransformListenerHandler.hpp"
#include "modulo_core/StateConversion.hpp"

using namespace std::chrono_literals;

namespace ModuloCore
{
	class Cell : public rclcpp_lifecycle::LifecycleNode
	{
	private:
		bool running_; ///< boolean that start or stop the main loop of the cell
		bool active_; ///< boolean that start computation only if the cell is in the active state
		std::thread run_thread; ///< thread object to start the main loop, i.e. the run function, in parallel of the rest
		std::shared_ptr<std::mutex> mutex_; /// A mutex to use when modifying messages between functions
		std::chrono::milliseconds period_;  ///< rate of the publisher functions in milliseconds
		std::map<std::string, std::shared_ptr<Communication::CommunicationHandler> > handlers_; ///< maps for storing publishers, subscriptions and tf 
		
		/**
		 * @brief Function to clear all publishers, subscriptions and services
		 */
		void reset();

		/**
		 * @brief Function to add a default transform broadcaster to the map of handlers
		 * @param period the period to wait between two publishing
		 * @param timeout the period after wich to consider that the publisher has timeout
		 */
		void add_transform_broadcaster(const std::chrono::milliseconds& period, const std::chrono::milliseconds& timeout, int queue_size=10)
		{
			auto handler = std::make_shared<Communication::TransformBroadcasterHandler>(timeout, this->get_clock(), this->mutex_);
			handler->set_publisher(this->create_publisher<tf2_msgs::msg::TFMessage>("tf", queue_size));
			handler->set_timer(this->create_wall_timer(period, std::bind(&Communication::TransformBroadcasterHandler::publish_callback, handler)));
			this->handlers_.insert(std::make_pair("tf_broadcaster", handler));
		}

		/**
		 * @brief Function to add a default transform listener to the map of handlers
		 * @param timeout the period after wich to consider that the handler has timeout
		 */
		void add_transform_listener(const std::chrono::milliseconds& timeout)
		{
			auto handler = std::make_shared<Communication::TransformListenerHandler>(timeout, this->get_clock(), this->mutex_);
			this->handlers_.insert(std::make_pair("tf_listener", handler));
		}

	protected:
		/**
		 * @brief Getter of the handlers variable
		 * @return Reference to the handlers variable
		 */
		inline const std::map<std::string, std::shared_ptr<Communication::CommunicationHandler> > & get_handlers() const
		{
			return this->handlers_;
		}


	public:
		/**
		 * @brief Cell constructor with arguments needed from ROS2 node
		 * @param node_name name of the ROS node
		 */
		explicit Cell(const std::string & node_name, const std::chrono::milliseconds & period, bool intra_process_comms = false);

		/**
		 * @brief Getter of the period variable
		 * @return Reference to the period variable
		 */
		inline std::chrono::milliseconds & get_period()
		{
			return this->period_;
		}

		/**
		 * @brief Getter of the period variable
		 * @return Reference to the period variable as a const reference
		 */
		inline const std::chrono::milliseconds & get_period() const
		{
			return this->period_;
		}

		/**
		 * @brief Template function to add a generic publisher to the map of handlers
		 * @param channel unique name of the publish channel that is used as key to the map
		 * @param recipient the state that contain the data to be published
		 * @param period the period to wait between two publishing
		 * @param timeout the period after wich to consider that the publisher has timeout
		 */
		template <typename MsgT, class RecT>
		void add_publisher(const std::string & channel, const std::shared_ptr<RecT>& recipient, const std::chrono::milliseconds& period, const std::chrono::milliseconds& timeout, int queue_size=10)
		{
			auto handler = std::make_shared<Communication::PublisherHandler<RecT, MsgT> >(channel, recipient, timeout, this->get_clock(), this->mutex_);
			handler->set_publisher(this->create_publisher<MsgT>(channel, queue_size));
			handler->set_timer(this->create_wall_timer(period, std::bind(&Communication::PublisherHandler<RecT, MsgT>::publish_callback, handler)));
			this->handlers_.insert(std::make_pair(channel, handler));
		}

		/**
		 * @brief Template function to add a generic publisher to the map of handlers
		 * @param channel unique name of the publish channel that is used as key to the map
		 * @param recipient the state that contain the data to be published
		 * @param timeout the period after wich to consider that the publisher has timeout
		 */
		template <typename MsgT, class RecT>
		void add_publisher(const std::string & channel, const std::shared_ptr<RecT>& recipient, const std::chrono::milliseconds& timeout, int queue_size=10)
		{
			this->add_publisher<MsgT, RecT>(channel, recipient, this->period_, timeout, queue_size);
		}

		/**
		 * @brief Template function to add a generic publisher to the map of handlers
		 * @param channel unique name of the publish channel that is used as key to the map
		 * @param recipient the state that contain the data to be published
		 */
		template <typename MsgT, class RecT>
		void add_publisher(const std::string & channel, const std::shared_ptr<RecT>& recipient, int queue_size=10)
		{
			this->add_publisher<MsgT, RecT>(channel, recipient, this->period_, 2*this->period_, queue_size);
		}

		/**
		 * @brief Function to add a generic transform broadcaster to the map of handlers
		 * @param recipient the state that contain the data to be published
		 * @param period the period to wait between two publishing
		 * @param timeout the period after wich to consider that the publisher has timeout
		 */
		void add_asynchronous_transform_broadcaster(const std::shared_ptr<StateRepresentation::CartesianPose>& recipient, const std::chrono::milliseconds& period, const std::chrono::milliseconds& timeout, int queue_size=10)
		{
			auto handler = std::make_shared<Communication::TransformBroadcasterHandler>(recipient, timeout, this->get_clock(), this->mutex_);
			handler->set_publisher(this->create_publisher<tf2_msgs::msg::TFMessage>("tf", queue_size));
			handler->set_timer(this->create_wall_timer(period, std::bind(&Communication::TransformBroadcasterHandler::publish_callback, handler)));
			this->handlers_.insert(std::make_pair(recipient->get_name() + "_in_" + recipient->get_reference_frame() + "_broadcaster", handler));
		}

		/**
		 * @brief Function to add a generic transform broadcaster to the map of handlers
		 * @param recipient the state that contain the data to be published
		 * @param timeout the period after wich to consider that the publisher has timeout
		 */
		void add_asynchronous_transform_broadcaster(const std::shared_ptr<StateRepresentation::CartesianPose>& recipient, const std::chrono::milliseconds& timeout, int queue_size=10)
		{
			this->add_asynchronous_transform_broadcaster(recipient, this->period_, timeout, queue_size);
		}

		/**
		 * @brief Function to add a generic transform broadcaster to the map of handlers
		 * @param recipient the state that contain the data to be published
		 */
		void add_asynchronous_transform_broadcaster(const std::shared_ptr<StateRepresentation::CartesianPose>& recipient, int queue_size=10)
		{
			this->add_asynchronous_transform_broadcaster(recipient, this->period_, 2*this->period_, queue_size);
		}

		/**
		 * @brief Function to add a fixed transform broadcaster to the map of handlers. A fixed transform broadcaster never times out as opposded to a normal broadcaster.
		 * @param recipient the state that contain the data to be published
		 * @param period the period to wait between two publishing
		 */
		void add_fixed_transform_broadcaster(const std::shared_ptr<StateRepresentation::CartesianPose>& recipient, const std::chrono::milliseconds& period, int queue_size=10)
		{
			this->add_asynchronous_transform_broadcaster(recipient, period, std::chrono::milliseconds(0), queue_size);
		}

		/**
		 * @brief Function to add a fixed transform broadcaster to the map of handlers. A fixed transform broadcaster never times out as opposded to a normal broadcaster.
		 * @param recipient the state that contain the data to be published
		 */
		void add_fixed_transform_broadcaster(const std::shared_ptr<StateRepresentation::CartesianPose>& recipient, int queue_size=10)
		{
			this->add_asynchronous_transform_broadcaster(recipient, this->period_, std::chrono::milliseconds(0), queue_size);
		}

		/**
		 * @brief Template function to add a generic subscription to the map of handlers
		 * @param channel unique name of the subscription channel that is used as key to the map
		 * @param recipient the state that will contain the received data
		 * @param timeout the period after wich to consider that the subscriber has timeout
		 */
		template <typename MsgT, class RecT>
		void add_subscription(const std::string & channel, const std::shared_ptr<RecT>& recipient, const std::chrono::milliseconds& timeout, int queue_size=10)
		{
			auto handler = std::make_shared<Communication::SubscriptionHandler<RecT, MsgT> >(channel, recipient, timeout, this->get_clock(), this->mutex_);
			handler->set_subscription(this->create_subscription<MsgT>(channel, queue_size, std::bind(&Communication::SubscriptionHandler<RecT, MsgT>::subscription_callback, handler, std::placeholders::_1)));
			this->handlers_.insert(std::make_pair(channel, handler));
		}

		/**
		 * @brief Template function to add a generic subscription to the map of handlers
		 * @param channel unique name of the subscription channel that is used as key to the map
		 * @param recipient the state that will contain the received data
		 */
		template <typename MsgT, class RecT>
		void add_subscription(const std::string & channel, const std::shared_ptr<RecT>& recipient, int queue_size=10)
		{
			this->add_subscription<MsgT, RecT>(channel, recipient, 2*this->period_, queue_size);
		}

		/**
		 * @brief Transition callback for state configuring
		 *
		 * on_configure callback is being called when the lifecycle node
		 * enters the "configuring" state.
		 * Depending on the return value of this function, the state machine
		 * either invokes a transition to the "inactive" state or stays
		 * in "unconfigured".
		 * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
		 * TRANSITION_CALLBACK_FAILURE transitions to "unconfigured"
		 * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
		 */
		virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);

		/**
		 * @brief Transition callback for state activating
		 *
		 * on_activate callback is being called when the lifecycle node
		 * enters the "activating" state.
		 * Depending on the return value of this function, the state machine
		 * either invokes a transition to the "active" state or stays
		 * in "inactive".
		 * TRANSITION_CALLBACK_SUCCESS transitions to "active"
		 * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
		 * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
		 */
		virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);

		/**
		 * @brief Transition callback for state deactivating
		 *
		 * on_deactivate callback is being called when the lifecycle node
		 * enters the "deactivating" state.
		 * Depending on the return value of this function, the state machine
		 * either invokes a transition to the "inactive" state or stays
		 * in "active".
		 * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
		 * TRANSITION_CALLBACK_FAILURE transitions to "active"
		 * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
		 */
		virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);

		/**
		 * @brief Transition callback for state cleaningup
		 *
		 * on_cleanup callback is being called when the lifecycle node
		 * enters the "cleaningup" state.
		 * Depending on the return value of this function, the state machine
		 * either invokes a transition to the "unconfigured" state or stays
		 * in "inactive".
		 * TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
		 * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
		 * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
		 */
		virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);

		/**
		 * @brief Transition callback for state shutting down
		 *
		 * on_shutdown callback is being called when the lifecycle node
		 * enters the "shuttingdown" state.
		 * Depending on the return value of this function, the state machine
		 * either invokes a transition to the "finalized" state or stays
		 * in its current state.
		 * TRANSITION_CALLBACK_SUCCESS transitions to "finalized"
		 * TRANSITION_CALLBACK_FAILURE transitions to current state
		 * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
		 */
		virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);

		/**
		 * @brief Main loop that will be executed in parallel of the rest. At each time step it calls the step function.
		 */
		void run();

		/**
		 * @brief Function to send a transform using the generic transform broadcaster
		 * @param transform the transformation to send
		 */
		void send_transform(const StateRepresentation::CartesianPose& transform);

		/**
		 * @brief Function to get a transform from the generic transform listener
		 * @param frame_name name of the frame to look for
		 * @param the frame in wich to express the transform
		 * @return the CartesianPose representing the tranformation
		 */
		const StateRepresentation::CartesianPose lookup_transform(const std::string& frame_name, const std::string& reference_frame="world");

		/**
		 * @brief Function computing one step of calculation. It is called periodically in the run function.
		 */
		virtual void step() = 0;
	};
}
#endif

