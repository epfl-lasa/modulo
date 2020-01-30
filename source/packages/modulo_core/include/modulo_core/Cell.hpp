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
#include <list>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp/function_traits.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rcutils/logging_macros.h"
#include "modulo_core/Communication/SubscriptionHandler.hpp"
#include "modulo_core/Communication/PublisherHandler.hpp"
#include "modulo_core/Communication/TransformBroadcasterHandler.hpp"
#include "modulo_core/Communication/TransformListenerHandler.hpp"

using namespace std::chrono_literals;

namespace Modulo
{
	namespace Core
	{
		class Cell : public rclcpp_lifecycle::LifecycleNode
		{
		private:
			bool configured_; ///< boolean that informs that the node has been configured, i.e passed by the on_configure state
			bool active_; ///< boolean that informs that the node has been activated, i.e passed by the on_activate state
			bool shutdown_; ///< boolean that informs that the node has been shutdown, i.e passed by the on_shutdown state
			std::thread run_thread; ///< thread object to start the main loop, i.e. the run function, in parallel of the rest
			std::shared_ptr<std::mutex> mutex_; ///< a mutex to use when modifying messages between functions
			std::chrono::milliseconds period_;  ///< rate of the publisher functions in milliseconds
			std::map<std::string, std::shared_ptr<Communication::CommunicationHandler> > handlers_; ///< maps for storing publishers, subscriptions and tf 
			std::list<std::thread> active_threads_; ///< list of active threads for periodic calling
			std::map<std::string, bool> configure_on_parameters_change_; ///< map of bools to store the configure_on_change value of each parameters

			/**
			 * @brief Function to clear all publishers, subscriptions and services
			 */
			void reset();

			/**
			 * @brief Function to add a default transform broadcaster to the map of handlers
			 * @param period the period to wait between two publishing
			 * @param timeout the period after wich to consider that the publisher has timeout
			 */
			void add_transform_broadcaster(const std::chrono::milliseconds& period, const std::chrono::milliseconds& timeout, int queue_size=10);

			/**
			 * @brief Function to add a default transform listener to the map of handlers
			 * @param timeout the period after wich to consider that the handler has timeout
			 */
			void add_transform_listener(const std::chrono::milliseconds& timeout);

		protected:
			/**
			 * @brief Getter of the handlers attribute
			 * @return Reference to the handlers attribute
			 */
			const std::map<std::string, std::shared_ptr<Communication::CommunicationHandler> > & get_handlers() const;

			/**
			 * @brief Getter of the mutex attribute
			 * @return Reference to the mutex attribute
			 */
			std::mutex & get_mutex();

			/**
			 * @brief Getter of the configured boolean attribute
			 * @return true if configured
			 */
			bool is_configured() const;

			/**
			 * @brief Getter of the active boolean attribute
			 * @return true if active
			 */
			bool is_active() const;

			/**
			 * @brief Getter of the shutdown boolean attribute
			 * @return true if shutdown
			 */
			bool is_shutdown() const;

		public:

			std::shared_ptr<rclcpp::SyncParametersClient> parameters_client_; ///< shared pointer to the parameter client that handles request to the parameter server
			
			/**
			 * @brief Cell constructor with arguments needed from ROS2 node
			 * @param node_name name of the ROS node
			 */
			explicit Cell(const std::string & node_name, const std::chrono::milliseconds & period, bool intra_process_comms = false);

			/**
			 * @brief Destructor
			 */
			~Cell();

			/**
			 * @brief Getter of the period attribute
			 * @return Reference to the period attribute
			 */
			const std::chrono::milliseconds & get_period() const;

			/**
			 * @brief Template function to add a generic publisher to the map of handlers
			 * @param channel unique name of the publish channel that is used as key to the map
			 * @param recipient the state that contain the data to be published
			 * @param period the period to wait between two publishing
			 * @param timeout the period after wich to consider that the publisher has timeout
			 */
			template <typename MsgT, class RecT>
			void add_publisher(const std::string & channel, const std::shared_ptr<RecT>& recipient, const std::chrono::milliseconds& period, const std::chrono::milliseconds& timeout, int queue_size=10);

			/**
			 * @brief Template function to add a generic publisher to the map of handlers
			 * @param channel unique name of the publish channel that is used as key to the map
			 * @param recipient the state that contain the data to be published
			 * @param timeout the period after wich to consider that the publisher has timeout
			 */
			template <typename MsgT, class RecT>
			void add_publisher(const std::string & channel, const std::shared_ptr<RecT>& recipient, const std::chrono::milliseconds& timeout, int queue_size=10);

			/**
			 * @brief Template function to add a generic publisher to the map of handlers
			 * @param channel unique name of the publish channel that is used as key to the map
			 * @param recipient the state that contain the data to be published
			 * @param nb_period_to_timeout the number of period before considering that the publisher has timeout 
			 */
			template <typename MsgT, class RecT>
			void add_publisher(const std::string & channel, const std::shared_ptr<RecT>& recipient, unsigned int nb_period_to_timeout=10, int queue_size=10);

			/**
			 * @brief Template function to add a generic subscription to the map of handlers
			 * @param channel unique name of the subscription channel that is used as key to the map
			 * @param recipient the state that will contain the received data
			 * @param timeout the period after wich to consider that the subscriber has timeout
			 */
			template <typename MsgT, class RecT>
			void add_subscription(const std::string & channel, const std::shared_ptr<RecT>& recipient, const std::chrono::milliseconds& timeout, int queue_size=10);

			/**
			 * @brief Template function to add a generic subscription to the map of handlers
			 * @param channel unique name of the subscription channel that is used as key to the map
			 * @param recipient the state that will contain the received data
			 * @param nb_period_to_timeout the number of period before considering that the subscription has timeout 
			 */
			template <typename MsgT, class RecT>
			void add_subscription(const std::string & channel, const std::shared_ptr<RecT>& recipient, unsigned int nb_period_to_timeout=10, int queue_size=10);

			/**
			 * @brief Add a parameter on the node parameter server
			 * @param name the name of the parameter
			 * @param default_value the default value of the parameter
			 * @param configure_on_change if true deactivate the node and call 
			 * the on_configure state e.g. to change the topics of publisher/subcriptions
			 */
			template <typename T>
			void add_parameter(const std::string & name, const T & default_value, bool configure_on_change=false);

			/**
			 * @brief Set the value of the parameter on the node parameter server
			 * @param name the name of the parameter
			 */
			template <typename T>
			void set_parameter(const std::string & name, const T & value);

			/**
			 * @brief Get the value of the parameter on the node parameter server
			 * @param name the name of the parameter
			 */
			template <typename T>
			decltype(auto) get_parameter(const std::string & name) const;

			/**
			 * @brief Function to add a generic transform broadcaster to the map of handlers
			 * @param recipient the state that contain the data to be published
			 * @param period the period to wait between two publishing
			 * @param timeout the period after wich to consider that the publisher has timeout
			 */
			void add_asynchronous_transform_broadcaster(const std::shared_ptr<StateRepresentation::CartesianPose>& recipient, const std::chrono::milliseconds& period, const std::chrono::milliseconds& timeout, int queue_size=10);

			/**
			 * @brief Function to add a generic transform broadcaster to the map of handlers
			 * @param recipient the state that contain the data to be published
			 * @param timeout the period after wich to consider that the publisher has timeout
			 */
			void add_asynchronous_transform_broadcaster(const std::shared_ptr<StateRepresentation::CartesianPose>& recipient, const std::chrono::milliseconds& timeout, int queue_size=10);

			/**
			 * @brief Function to add a generic transform broadcaster to the map of handlers
			 * @param recipient the state that contain the data to be published
			 * @param nb_period_to_timeout the number of period before considering that the broadcaster has timeout 
			 */
			void add_asynchronous_transform_broadcaster(const std::shared_ptr<StateRepresentation::CartesianPose>& recipient, unsigned int nb_period_to_timeout=10, int queue_size=10);

			/**
			 * @brief Function to add a fixed transform broadcaster to the map of handlers. A fixed transform broadcaster never times out as opposded to a normal broadcaster.
			 * @param recipient the state that contain the data to be published
			 * @param period the period to wait between two publishing
			 */
			void add_fixed_transform_broadcaster(const std::shared_ptr<StateRepresentation::CartesianPose>& recipient, const std::chrono::milliseconds& period, int queue_size=10);
	
			/**
			 * @brief Function to add a fixed transform broadcaster to the map of handlers. A fixed transform broadcaster never times out as opposded to a normal broadcaster.
			 * @param recipient the state that contain the data to be published
			 */
			void add_fixed_transform_broadcaster(const std::shared_ptr<StateRepresentation::CartesianPose>& recipient, int queue_size=10);

			/**
			 * @brief Function to send a transform using the generic transform broadcaster
			 * @param transform the transformation to send
			 */
			void send_transform(const StateRepresentation::CartesianPose& transform);

			/**
			 * @brief Function to send a transform using the generic transform broadcaster
			 * @param transform the shared pointer to the transformation to send
			 */
			void send_transform(const std::shared_ptr<StateRepresentation::CartesianPose>& transform);

			/**
			 * @brief Function to get a transform from the generic transform listener
			 * @param frame_name name of the frame to look for
			 * @param the frame in wich to express the transform
			 * @return the CartesianPose representing the tranformation
			 */
			const StateRepresentation::CartesianPose lookup_transform(const std::string& frame_name, const std::string& reference_frame="world");

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
			rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);

			/**
			 * @brief Proxy function for the on_configure ROS2 lifecycle function.
			 * This function is called by the main on_configure function and is made to
			 * adapted to the derived class.
			 */
			virtual void on_configure();

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
			rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);

			/**
			 * @brief Proxy function for the on_activate ROS2 lifecycle function.
			 * This function is called by the main on_activate function and is made to
			 * adapted to the derived class.
			 */
			virtual void on_activate();

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
			rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);

			/**
			 * @brief Proxy function for the on_deactivate ROS2 lifecycle function.
			 * This function is called by the main on_deactivate function and is made to
			 * adapted to the derived class.
			 */
			virtual void on_deactivate();

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
			rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);

			/**
			 * @brief Proxy function for the on_cleanup ROS2 lifecycle function.
			 * This function is called by the main on_cleanup function and is made to
			 * adapted to the derived class.
			 */
			virtual void on_cleanup();

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
			rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);

			/**
			 * @brief Proxy function for the on_shutdown ROS2 lifecycle function.
			 * This function is called by the main on_shutdown function and is made to
			 * adapted to the derived class.
			 */
			virtual void on_shutdown();

			/**
			 * @brief Function called each time a a parameter is modified on the paramter server of the node
			 * @param event the type of event in parameter added, parameter changed or deleted
			 * @param logger logger of the node for printing
			 */
			void on_parameter_event(const rcl_interfaces::msg::ParameterEvent::SharedPtr event, rclcpp::Logger logger);

			/**
			 * @brief Function computing one step of calculation. It is called periodically in the run function.
			 */
			virtual void step() = 0;

			/**
			 * @brief Main loop that will be executed in parallel of the rest. At each time step it calls the step function.
			 */
			void run();

			/**
			 * @brief Function to periodically call the given callback_function at the given period
			 * @param callback_function the function to call
			 * @param period the period between two calls
			 */
			void run_periodic_call(const std::function<void(void)>& callback_function, const std::chrono::milliseconds& period);

			/**
			 * @brief Function to add a periodic call to the function given in input
			 * @param callback_function the function to call
			 * @param period the period between two calls
			 */
			void add_periodic_call(const std::function<void(void)>& callback_function, const std::chrono::milliseconds& period);
		};

		inline const std::map<std::string, std::shared_ptr<Communication::CommunicationHandler> > & Cell::get_handlers() const
		{
			return this->handlers_;
		}

		inline bool Cell::is_configured() const
		{
			return this->configured_;
		}

		inline bool Cell::is_active() const
		{
			return this->active_;
		}

		inline bool Cell::is_shutdown() const
		{
			return this->shutdown_;
		}

		inline std::mutex & Cell::get_mutex()
		{
			return (*this->mutex_);
		}

		inline const std::chrono::milliseconds & Cell::get_period() const
		{
			return this->period_;
		}

		template <typename MsgT, class RecT>
		void Cell::add_publisher(const std::string & channel, const std::shared_ptr<RecT>& recipient, const std::chrono::milliseconds& period, const std::chrono::milliseconds& timeout, int queue_size)
		{
			auto handler = std::make_shared<Communication::PublisherHandler<RecT, MsgT> >(channel, recipient, timeout, this->get_clock(), this->mutex_);
			handler->set_publisher(this->create_publisher<MsgT>(channel, queue_size));
			handler->set_timer(this->create_wall_timer(period, std::bind(&Communication::PublisherHandler<RecT, MsgT>::publish_callback, handler)));
			this->handlers_.insert(std::make_pair(channel, handler));
		}

		template <typename MsgT, class RecT>
		void Cell::add_publisher(const std::string & channel, const std::shared_ptr<RecT>& recipient, const std::chrono::milliseconds& timeout, int queue_size)
		{
			this->add_publisher<MsgT, RecT>(channel, recipient, this->period_, timeout, queue_size);
		}

		template <typename MsgT, class RecT>
		void Cell::add_publisher(const std::string & channel, const std::shared_ptr<RecT>& recipient, unsigned int nb_period_to_timeout, int queue_size)
		{
			this->add_publisher<MsgT, RecT>(channel, recipient, this->period_, nb_period_to_timeout*this->period_, queue_size);
		}

		template <typename MsgT, class RecT>
		void Cell::add_subscription(const std::string & channel, const std::shared_ptr<RecT>& recipient, const std::chrono::milliseconds& timeout, int queue_size)
		{
			auto handler = std::make_shared<Communication::SubscriptionHandler<RecT, MsgT> >(channel, recipient, timeout, this->get_clock(), this->mutex_);
			handler->set_subscription(this->create_subscription<MsgT>(channel, queue_size, std::bind(&Communication::SubscriptionHandler<RecT, MsgT>::subscription_callback, handler, std::placeholders::_1)));
			this->handlers_.insert(std::make_pair(channel, handler));
		}

		template <typename MsgT, class RecT>
		void Cell::add_subscription(const std::string & channel, const std::shared_ptr<RecT>& recipient, unsigned int nb_period_to_timeout, int queue_size)
		{
			this->add_subscription<MsgT, RecT>(channel, recipient, nb_period_to_timeout*this->period_, queue_size);
		}

		template <typename T>
		void Cell::add_parameter(const std::string & name, const T & default_value, bool configure_on_change)
		{
			this->configure_on_parameters_change_.insert(std::make_pair(name, configure_on_change));
			this->declare_parameter(name, rclcpp::ParameterValue(default_value));
		}

		template <typename T>
		void Cell::set_parameter(const std::string & name, const T & value)
		{
			auto result = this->parameters_client_->set_parameters({rclcpp::Parameter(name, value)});
			if (!result[0].successful) 
			{
        		RCLCPP_ERROR(this->get_logger(), "Failed to set parameter: %s", result[0].reason.c_str());
      		}
		}

		template <typename T>
		decltype(auto) Cell::get_parameter(const std::string & name) const
		{
			auto parameter = this->parameters_client_->get_parameters({name});
			return parameter[0].get_value<T>();
		}
	}
}
#endif

