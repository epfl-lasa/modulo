#include "modulo_core/Cell.hpp"
#include "modulo_core/Exceptions/UnconfiguredNodeException.hpp"

using namespace Modulo::Exceptions;

namespace Modulo
{
	namespace Core
	{
		Cell::~Cell()
		{
			this->on_shutdown();
		}

		void Cell::reset()
		{
			this->active_ = false;
			this->configured_ = false;
			this->handlers_.clear();
			this->parameters_.clear();
			for(auto& t:this->active_threads_)
			{
				t.join();
			}
			this->active_threads_.clear();
			this->run_thread_.join();
		}

		void Cell::add_asynchronous_transform_broadcaster(const std::shared_ptr<StateRepresentation::CartesianState>& recipient, unsigned int nb_period_to_timeout, int queue_size)
		{
			this->add_asynchronous_transform_broadcaster(recipient, this->period_, nb_period_to_timeout * this->period_, queue_size);
		}

		void Cell::add_fixed_transform_broadcaster(const std::shared_ptr<StateRepresentation::CartesianState>& recipient, int queue_size)
		{
			this->add_asynchronous_transform_broadcaster(recipient, this->period_, 0ms, queue_size);
		}

		template<>
		void Cell::add_parameter(const std::shared_ptr<StateRepresentation::Parameter<StateRepresentation::CartesianState>>& parameter)
		{
			this->parameters_.push_back(parameter);
			this->declare_parameter<std::vector<double>>(parameter->get_name(), parameter->get_value().to_std_vector());
		}

		void Cell::send_transform(const StateRepresentation::CartesianState& transform)
		{
			if (!this->configured_) throw UnconfiguredNodeException("The node is not yet configured. Call the lifecycle configure before using this function");
			static_cast<Communication::MessagePassing::TransformBroadcasterHandler&>(*this->handlers_["tf_broadcaster"]).send_transform(transform);
		}

		const StateRepresentation::CartesianPose Cell::lookup_transform(const std::string& frame_name, const std::string& reference_frame)
		{
			if (!this->configured_) throw UnconfiguredNodeException("The node is not yet configured. Call the lifecycle configure before using this function");
			return static_cast<Communication::MessagePassing::TransformListenerHandler&>(*this->handlers_["tf_listener"]).lookup_transform(frame_name, reference_frame);
		}

		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Cell::on_configure(const rclcpp_lifecycle::State &) 
		{
			RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");
			//std::lock_guard<std::mutex> lock(*this->mutex_);
			// call the proxy on_configure function
			if(!this->on_configure())
			{
				RCLCPP_ERROR(get_logger(), "Configuration failed");
				this->reset();
				return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
			}
			// add the run thread
			std::function<void(void)> run_fnc = std::bind(&Cell::run, this);
			this->run_thread_ = std::thread(run_fnc);
			// start the parameters
			this->active_ = false;
			this->configured_ = true;
			// add default transform broadcaster and transform listener
			this->add_transform_broadcaster(this->period_, 10*this->period_);
			this->add_transform_listener(10*this->period_);
			return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
		}

		bool Cell::on_configure()
		{
			RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure of the Cell class called");
			return true;
		}

		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Cell::on_activate(const rclcpp_lifecycle::State &)
		{
			RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
			//std::lock_guard<std::mutex> lock(*this->mutex_);
			// call the proxy on_activate function
			if(!this->on_activate())
			{
				RCLCPP_ERROR(get_logger(), "Activation failed.");
				return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
			}
			// set all handlers to activated
			this->active_ = true;
			for (auto &h : this->handlers_)
			{
				h.second->activate();
			}
			return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
		}

		bool Cell::on_activate()
		{
			RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate of the Cell class called");
			return true;
		}

		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Cell::on_deactivate(const rclcpp_lifecycle::State &)
		{
			RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
			//std::lock_guard<std::mutex> lock(*this->mutex_);
			// call the proxy on_deactivate function
			if(!this->on_deactivate())
			{
				RCLCPP_ERROR(get_logger(), "Deactivation failed.");
				return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
			}
			// set all handlers to not activated
			this->active_ = false;
			for (auto &h : this->handlers_)
			{
				h.second->deactivate();
			}
			return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
		}

		bool Cell::on_deactivate()
		{
			RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate of the Cell class called");
			return true;
		}

		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Cell::on_cleanup(const rclcpp_lifecycle::State &) 
		{
			RCUTILS_LOG_INFO_NAMED(get_name(), "on_cleanup() is called.");
			//std::lock_guard<std::mutex> lock(*this->mutex_);
			// call the proxy on_cleanup function
			if(!this->on_cleanup())
			{
				RCLCPP_ERROR(get_logger(), "Cleanup failed.");
				return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
			}
			// reset all handlers
			this->reset();
			return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
		}

		bool Cell::on_cleanup()
		{
			RCUTILS_LOG_INFO_NAMED(get_name(), "on_cleanup of the Cell class called");
			return true;
		}

		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Cell::on_shutdown(const rclcpp_lifecycle::State & state) 
		{
			RCUTILS_LOG_INFO_NAMED(get_name(), "on_shutdown() is called from state %s.", state.label().c_str());
			//std::lock_guard<std::mutex> lock(*this->mutex_);
			// call the proxy on_shutdown function
			if(!this->on_shutdown())
			{
				RCLCPP_ERROR(get_logger(), "Shutdown failed.");
				return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
			}
			// reset all handlers for clean shutdown
			this->reset();
			this->shutdown_ = true;
			return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
		}

		bool Cell::on_shutdown()
		{
			RCUTILS_LOG_INFO_NAMED(get_name(), "on_shutdown of the Cell class called");
			return true;
		}

		void Cell::run()
		{
			while(this->configured_)
			{
				auto start = std::chrono::steady_clock::now();
				std::unique_lock<std::mutex> lck(*this->mutex_);
				for (auto &h : this->handlers_)
				{
					h.second->check_timeout();
				}
				if(this->active_)
				{
					this->step();
				}
				lck.unlock();
				auto end = std::chrono::steady_clock::now();
		    	auto elapsed = end - start;
		    	auto timeToWait = this->period_ - elapsed;
		    	if(timeToWait > std::chrono::nanoseconds::zero())
		    	{
		        	std::this_thread::sleep_for(timeToWait);
		    	}
		    }
		}

		void Cell::step()
		{}
	}
}


