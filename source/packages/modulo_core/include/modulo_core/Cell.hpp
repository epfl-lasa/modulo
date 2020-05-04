/**
 * @author Baptiste Busch
 * @date 2019/02/14
 */

#pragma once

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <list>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp/function_traits.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rcutils/logging_macros.h>
#include "modulo_core/Communication/MessagePassing/SubscriptionHandler.hpp"
#include "modulo_core/Communication/MessagePassing/PublisherHandler.hpp"
#include "modulo_core/Communication/MessagePassing/TransformBroadcasterHandler.hpp"
#include "modulo_core/Communication/MessagePassing/TransformListenerHandler.hpp"
#include "modulo_core/Communication/ServiceClient/ClientHandler.hpp"
#include "state_representation/Parameters/Parameter.hpp"

using namespace std::chrono_literals;

namespace Modulo
{
	namespace Core
	{
		/**
		 * @class Cell
 		 * @brief Abstract class to define a Call
		 *
		 * A Cell is the base class of the whole architecture.
		 * It handles all the basic ROS communications such as
		 * definitions of subrscriptions, publishers and service
		 * calls. It can then be derived into a MotionGenerator,
		 * a Controller, a Sensor, or a RobotInterface. It is
		 * derived from a lifecyle node which allows to use
		 * ROS2 state machine functionnalities for nodes.
		 */
		class Cell : public rclcpp_lifecycle::LifecycleNode
		{
		private:
			bool configured_; ///< boolean that informs that the node has been configured, i.e passed by the on_configure state
			bool active_; ///< boolean that informs that the node has been activated, i.e passed by the on_activate state
			bool shutdown_; ///< boolean that informs that the node has been shutdown, i.e passed by the on_shutdown state
			std::thread run_thread_; ///< thread object to start the main loop, i.e. the run function, in parallel of the rest
			std::shared_ptr<std::mutex> mutex_; ///< a mutex to use when modifying messages between functions
			std::chrono::nanoseconds period_;  ///< rate of the publisher functions in nanoseconds
			std::list<std::thread> active_threads_; ///< list of active threads for periodic calling
			std::list<std::shared_ptr<StateRepresentation::State>> parameters_; ///< list for storing parameters
			std::map<std::string, std::shared_ptr<Communication::CommunicationHandler> > handlers_; ///< map for storing publishers, subscriptions and tf 

			/**
			 * @brief Function to clear all publishers, subscriptions and services
			 */
			void reset();

			/**
			 * @brief Function to add a default transform broadcaster to the map of handlers
			 * @tparam DurationT template value for accepting any type of std::chrono duration values
			 * @param period the period to wait between two publishing
			 * @param timeout the period after wich to consider that the publisher has timeout
			 */
			template <typename DurationT>
			void add_transform_broadcaster(const std::chrono::duration<int64_t, DurationT>& period, const std::chrono::duration<int64_t, DurationT>& timeout, int queue_size=10);

			/**
			 * @brief Function to add a default transform listener to the map of handlers
			 * @tparam DurationT template value for accepting any type of std::chrono duration values
			 * @param timeout the period after wich to consider that the handler has timeout
			 */
			template <typename DurationT>
			void add_transform_listener(const std::chrono::duration<int64_t, DurationT>& timeout);

			/**
			 * @brief Update the value of the parameters from the parameter server
			 */
			void update_parameters();

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

		public:
			
			/**
			 * @brief Cell constructor with arguments needed from ROS2 node
			 * @tparam DurationT template value for accepting any type of std::chrono duration values
			 * @param node_name name of the ROS node
			 * @param period the period of each step function call
			 * @param intra_process_comms ROS2 parameter to declare if nodes share the same memory for instant process communication
			 */
			template <typename DurationT>
			explicit Cell(const std::string& node_name, const std::chrono::duration<int64_t, DurationT>& period, bool intra_process_comms = false);

			/**
			 * @brief Destructor
			 */
			~Cell();

			/**
			 * @brief Getter of the period attribute
			 * @return Reference to the period attribute
			 */
			const std::chrono::nanoseconds& get_period() const;

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

			/**
			 * @brief Template function to add a generic publisher to the map of handlers
			 * @tparam MsgT template value for accepting any type of ROS2 messages
			 * @tparam RecT template value for accepting any type of recipient
			 * @tparam DurationT template value for accepting any type of std::chrono duration values
			 * @param channel unique name of the publish channel that is used as key to the map
			 * @param recipient the state that contain the data to be published
			 * @param period the period to wait between two publishing
			 * @param timeout the period after wich to consider that the publisher has timeout
			 */
			template <typename MsgT, class RecT, typename DurationT1, typename DurationT2>
			void add_publisher(const std::string& channel, const std::shared_ptr<RecT>& recipient, const std::chrono::duration<int64_t, DurationT1>& period, const std::chrono::duration<int64_t, DurationT2>& timeout, int queue_size=10);

			/**
			 * @brief Template function to add a generic publisher to the map of handlers
			 * @tparam MsgT template value for accepting any type of ROS2 messages
			 * @tparam RecT template value for accepting any type of recipient
			 * @tparam DurationT template value for accepting any type of std::chrono duration values
			 * @param channel unique name of the publish channel that is used as key to the map
			 * @param recipient the state that contain the data to be published
			 * @param period the period to wait between two publishing
			 * @param nb_period_to_timeout the number of period (of the node) before considering that the publisher has timeout (default = 10, 0 means never timeout)
			 */
			template <typename MsgT, class RecT, typename DurationT>
			void add_publisher(const std::string& channel, const std::shared_ptr<RecT>& recipient, const std::chrono::duration<int64_t, DurationT>& period, unsigned int nb_period_to_timeout=10, int queue_size=10);

			/**
			 * @brief Template function to add a generic publisher to the map of handlers
			 * @tparam MsgT template value for accepting any type of ROS2 messages
			 * @tparam RecT template value for accepting any type of recipient
			 * @param channel unique name of the publish channel that is used as key to the map
			 * @param recipient the state that contain the data to be published
			 * @param nb_period_to_timeout the number of period (of the node) before considering that the publisher has timeout (default = 10, 0 means never timeout)
			 */
			template <typename MsgT, class RecT>
			void add_publisher(const std::string& channel, const std::shared_ptr<RecT>& recipient, unsigned int nb_period_to_timeout=10, int queue_size=10);

			/**
			 * @brief Template function to add a generic subscription to the map of handlers
			 * @tparam MsgT template value for accepting any type of ROS2 messages
			 * @tparam RecT template value for accepting any type of recipient
			 * @tparam DurationT template value for accepting any type of std::chrono duration values
			 * @param channel unique name of the subscription channel that is used as key to the map
			 * @param recipient the state that will contain the received data
			 * @param timeout the period after wich to consider that the subscriber has timeout
			 */
			template <typename MsgT, class RecT, typename DurationT>
			void add_subscription(const std::string& channel, const std::shared_ptr<RecT>& recipient, const std::chrono::duration<int64_t, DurationT>& timeout, int queue_size=10);

			/**
			 * @brief Template function to add a generic subscription to the map of handlers
			 * @param channel unique name of the subscription channel that is used as key to the map
			 * @param recipient the state that will contain the received data
			 * @param nb_period_to_timeout the number of period before considering that the subscription has timeout 
			 */
			template <typename MsgT, class RecT>
			void add_subscription(const std::string& channel, const std::shared_ptr<RecT>& recipient, unsigned int nb_period_to_timeout=10, int queue_size=10);

			/**
			 * @brief Template function to add a generic client to the map of handlers
			 * @tparam srvT tamplate value to accept any type of ROS2 services
			 * @tparam DurationT template value for accepting any type of std::chrono duration values
			 * @param channel unique name of the communication topic between the client and the server
			 * @param timeout period before considering the server is not responding
			 */
			template <typename srvT, typename DurationT>
			void add_client(const std::string& channel, const std::chrono::duration<int64_t, DurationT>& timeout);

			/**
			 * @brief Function to add a generic transform broadcaster to the map of handlers
			 * @param recipient the state that contain the data to be published
			 * @param period the period to wait between two publishing
			 * @param timeout the period after wich to consider that the publisher has timeout
			 */
			template <typename DurationT1, typename DurationT2>
			void add_asynchronous_transform_broadcaster(const std::shared_ptr<StateRepresentation::CartesianState>& recipient, const std::chrono::duration<int64_t, DurationT1>& period, const std::chrono::duration<int64_t, DurationT2>& timeout, int queue_size=10);

			/**
			 * @brief Function to add a generic transform broadcaster to the map of handlers
			 * @param recipient the state that contain the data to be published
			 * @param period the period to wait between two publishing
			 * @param nb_period_to_timeout the number of period (of the node) before considering that the publisher has timeout (default = 10, 0 means never timeout)
			 */
			template <typename DurationT>
			void add_asynchronous_transform_broadcaster(const std::shared_ptr<StateRepresentation::CartesianState>& recipient, const std::chrono::duration<int64_t, DurationT>& period, unsigned int nb_period_to_timeout=10, int queue_size=10);

			/**
			 * @brief Function to add a generic transform broadcaster to the map of handlers
			 * @param recipient the state that contain the data to be published
			 * @param nb_period_to_timeout the number of period (of the node) before considering that the broadcaster has timeout 
			 */
			void add_asynchronous_transform_broadcaster(const std::shared_ptr<StateRepresentation::CartesianState>& recipient, unsigned int nb_period_to_timeout=10, int queue_size=10);

			/**
			 * @brief Function to add a fixed transform broadcaster to the map of handlers. A fixed transform broadcaster never times out as opposded to a normal broadcaster.
			 * @param recipient the state that contain the data to be published
			 * @param period the period to wait between two publishing
			 */
			template <typename DurationT>
			void add_fixed_transform_broadcaster(const std::shared_ptr<StateRepresentation::CartesianState>& recipient, const std::chrono::duration<int64_t, DurationT>& period, int queue_size=10);
	
			/**
			 * @brief Function to add a fixed transform broadcaster to the map of handlers. A fixed transform broadcaster never times out as opposded to a normal broadcaster.
			 * @param recipient the state that contain the data to be published
			 */
			void add_fixed_transform_broadcaster(const std::shared_ptr<StateRepresentation::CartesianState>& recipient, int queue_size=10);

			/**
			 * @brief Add a parameter to be updated by the parameter server
			 * @brief The parameter using the StateRepresentation of a parameter
			 */
			template <typename T>
			void add_parameter(const std::shared_ptr<StateRepresentation::Parameter<T>>& parameter);

			/**
			 * @brief Function to send a transform using the generic transform broadcaster
			 * @param transform the transformation to send
			 */
			void send_transform(const StateRepresentation::CartesianState& transform);

			/**
			 * @brief Send a request to the server and wait for its response
			 * @param channel the channel of communication
			 * @param request the request to send
			 * @return the response from the server
			 */
			template <typename srvT>
			std::shared_ptr<typename srvT::Response> send_blocking_request(const std::string& channel, const  std::shared_ptr<typename srvT::Request>& request);

			/**
			 * @brief Send a request to the server without waiting for its response
			 * @param channel the channel of communication
			 * @param request the request to send
			 * @return the response from the server
			 */
			template <typename srvT>
			std::shared_future<std::shared_ptr<typename srvT::Response> > send_request(const std::string& channel, const  std::shared_ptr<typename srvT::Request>& request);

			/**
			 * @brief Function to get a transform from the generic transform listener
			 * @param frame_name name of the frame to look for
			 * @param the frame in wich to express the transform
			 * @return the CartesianPose representing the transformation
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
			rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

			/**
			 * @brief Proxy function for the on_configure ROS2 lifecycle function.
			 * This function is called by the main on_configure function and is made to
			 * adapted to the derived class.
			 */
			virtual bool on_configure();

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
			rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

			/**
			 * @brief Proxy function for the on_activate ROS2 lifecycle function.
			 * This function is called by the main on_activate function and is made to
			 * adapted to the derived class.
			 */
			virtual bool on_activate();

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
			rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

			/**
			 * @brief Proxy function for the on_deactivate ROS2 lifecycle function.
			 * This function is called by the main on_deactivate function and is made to
			 * adapted to the derived class.
			 */
			virtual bool on_deactivate();

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
			rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

			/**
			 * @brief Proxy function for the on_cleanup ROS2 lifecycle function.
			 * This function is called by the main on_cleanup function and is made to
			 * adapted to the derived class.
			 */
			virtual bool on_cleanup();

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
			rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

			/**
			 * @brief Proxy function for the on_shutdown ROS2 lifecycle function.
			 * This function is called by the main on_shutdown function and is made to
			 * adapted to the derived class.
			 */
			virtual bool on_shutdown();

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
			void run_periodic_call(const std::function<void(void)>& callback_function, const std::chrono::nanoseconds& period);

			/**
			 * @brief Function to add a periodic call to the function given in input
			 * @param callback_function the function to call
			 * @param period the period between two calls
			 */
			void add_periodic_call(const std::function<void(void)>& callback_function, const std::chrono::nanoseconds& period);
		};

		template <typename DurationT>
		Cell::Cell(const std::string & node_name, const std::chrono::duration<int64_t, DurationT> & period, bool intra_process_comms) :
		rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)),
		configured_(false),
		active_(false),
		shutdown_(false),
		mutex_(std::make_shared<std::mutex>()),
		period_(period)
		{}

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

		inline const std::chrono::nanoseconds& Cell::get_period() const
		{
			return this->period_;
		}

		template <typename MsgT, class RecT, typename DurationT1, typename DurationT2>
		void Cell::add_publisher(const std::string& channel, const std::shared_ptr<RecT>& recipient, const std::chrono::duration<int64_t, DurationT1>& period, const std::chrono::duration<int64_t, DurationT2>& timeout, int queue_size)
		{
			auto handler = std::make_shared<Communication::MessagePassing::PublisherHandler<RecT, MsgT> >(recipient, timeout, this->get_clock(), this->mutex_);
			handler->set_publisher(this->create_publisher<MsgT>(channel, queue_size));
			handler->set_timer(this->create_wall_timer(period, std::bind(&Communication::MessagePassing::PublisherHandler<RecT, MsgT>::publish_callback, handler)));
			this->handlers_.insert(std::make_pair(channel, handler));
		}

		template <typename MsgT, class RecT, typename DurationT>
		void Cell::add_publisher(const std::string& channel, const std::shared_ptr<RecT>& recipient, const std::chrono::duration<int64_t, DurationT>& period, unsigned int nb_period_to_timeout, int queue_size)
		{
			this->add_publisher<MsgT, RecT>(channel, recipient, period, nb_period_to_timeout*this->period_, queue_size);
		}

		template <typename MsgT, class RecT>
		void Cell::add_publisher(const std::string& channel, const std::shared_ptr<RecT>& recipient, unsigned int nb_period_to_timeout, int queue_size)
		{
			this->add_publisher<MsgT, RecT>(channel, recipient, this->period_, nb_period_to_timeout * this->period_, queue_size);
		}

		template <typename MsgT, class RecT, typename DurationT>
		void Cell::add_subscription(const std::string& channel, const std::shared_ptr<RecT>& recipient, const std::chrono::duration<int64_t, DurationT>& timeout, int queue_size)
		{
			auto handler = std::make_shared<Communication::MessagePassing::SubscriptionHandler<RecT, MsgT> >(recipient, timeout, this->mutex_);
			handler->set_subscription(this->create_subscription<MsgT>(channel, queue_size, std::bind(&Communication::MessagePassing::SubscriptionHandler<RecT, MsgT>::subscription_callback, handler, std::placeholders::_1)));
			this->handlers_.insert(std::make_pair(channel, handler));
		}

		template <typename MsgT, class RecT>
		void Cell::add_subscription(const std::string& channel, const std::shared_ptr<RecT>& recipient, unsigned int nb_period_to_timeout, int queue_size)
		{
			this->add_subscription<MsgT, RecT>(channel, recipient, nb_period_to_timeout * this->period_, queue_size);
		}

		template <typename DurationT>
		void Cell::add_transform_broadcaster(const std::chrono::duration<int64_t, DurationT>& period, const std::chrono::duration<int64_t, DurationT>& timeout, int queue_size)
		{
			auto handler = std::make_shared<Communication::MessagePassing::TransformBroadcasterHandler>(timeout, this->get_clock(), this->mutex_);
			handler->set_publisher(this->create_publisher<tf2_msgs::msg::TFMessage>("tf", queue_size));
			handler->set_timer(this->create_wall_timer(period, std::bind(&Communication::MessagePassing::TransformBroadcasterHandler::publish_callback, handler)));
			this->handlers_.insert(std::make_pair("tf_broadcaster", handler));
		}

		template <typename DurationT>
		void Cell::add_transform_listener(const std::chrono::duration<int64_t, DurationT>& timeout)
		{
			auto handler = std::make_shared<Communication::MessagePassing::TransformListenerHandler>(timeout, this->get_clock(), this->mutex_);
			this->handlers_.insert(std::make_pair("tf_listener", handler));
		}

		template <typename DurationT1, typename DurationT2>
		void Cell::add_asynchronous_transform_broadcaster(const std::shared_ptr<StateRepresentation::CartesianState>& recipient, const std::chrono::duration<int64_t, DurationT1>& period, const std::chrono::duration<int64_t, DurationT2>& timeout, int queue_size)
		{
			auto handler = std::make_shared<Communication::MessagePassing::TransformBroadcasterHandler>(recipient, timeout, this->get_clock(), this->mutex_);
			handler->set_publisher(this->create_publisher<tf2_msgs::msg::TFMessage>("tf", queue_size));
			handler->set_timer(this->create_wall_timer(period, std::bind(&Communication::MessagePassing::TransformBroadcasterHandler::publish_callback, handler)));
			this->handlers_.insert(std::make_pair(recipient->get_name() + "_in_" + recipient->get_reference_frame() + "_broadcaster", handler));
		}

		template <typename DurationT>
		void Cell::add_asynchronous_transform_broadcaster(const std::shared_ptr<StateRepresentation::CartesianState>& recipient, const std::chrono::duration<int64_t, DurationT>& period, unsigned int nb_period_to_timeout, int queue_size)
		{
			this->add_asynchronous_transform_broadcaster(recipient, period, nb_period_to_timeout * this->period_, queue_size);
		}

		template <typename DurationT>
		void Cell::add_fixed_transform_broadcaster(const std::shared_ptr<StateRepresentation::CartesianState>& recipient, const std::chrono::duration<int64_t, DurationT>& period, int queue_size)
		{
			this->add_asynchronous_transform_broadcaster(recipient, period, std::chrono::duration<int64_t, DurationT>(0), queue_size);
		}

		template <typename srvT, typename DurationT>
		void Cell::add_client(const std::string& channel, const std::chrono::duration<int64_t, DurationT>& timeout)
		{
			auto handler = std::make_shared<Communication::ServiceClient::ClientHandler<srvT> >(timeout, this->mutex_);
			handler->set_client(this->create_client<srvT>(channel));
			this->handlers_.insert(std::make_pair(channel, handler));
		}

		template <typename T>
		void Cell::add_parameter(const std::shared_ptr<StateRepresentation::Parameter<T>>& parameter)
		{
			this->parameters_.push_back(parameter);
			this->declare_parameter(parameter->get_name(), parameter->get_value());
		}

		template <typename srvT>
		std::shared_ptr<typename srvT::Response> Cell::send_blocking_request(const std::string& channel, const  std::shared_ptr<typename srvT::Request>& request)
		{
  			return static_cast<Communication::ServiceClient::ClientHandler<srvT>& >(*this->handlers_.at(channel)).send_blocking_request(request);
		}

		template <typename srvT>
		std::shared_future<std::shared_ptr<typename srvT::Response> > Cell::send_request(const std::string& channel, const  std::shared_ptr<typename srvT::Request>& request)
		{
			return static_cast<Communication::ServiceClient::ClientHandler<srvT>& >(*this->handlers_.at(channel)).send_request(request);
		}
	}
}
