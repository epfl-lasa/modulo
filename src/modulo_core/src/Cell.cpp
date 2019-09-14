#include "modulo_core/Cell.hpp"
#include "modulo_core/Exceptions/UnconfiguredNodeException.hpp"

using namespace Modulo::Exceptions;

namespace Modulo
{
	namespace Core
	{
		Cell::Cell(const std::string & node_name, const std::chrono::milliseconds & period, bool intra_process_comms) :
		rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)),
		running_(false),
		active_(false),
		shutdown_(false),
		mutex_(std::make_shared<std::mutex>()),
		period_(period)
		{
			std::function<void(void)> run_fnc = std::bind(&Cell::run, this);
			this->run_thread = std::thread(run_fnc);
		}

		Cell::~Cell()
		{
			this->on_shutdown();
		}

		void Cell::reset()
		{
			this->handlers_.clear();
		}

		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Cell::on_configure(const rclcpp_lifecycle::State &) 
		{
			RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");
			std::lock_guard<std::mutex> lock(*this->mutex_);
			this->active_ = false;
			this->running_ = true;
			// add default transform broadcaster and transform listener
			this->add_transform_broadcaster(this->period_, 10*this->period_);
			this->add_transform_listener(10*this->period_);
			// call the proxy on_configure function
			this->on_configure();
			return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
		}

		void Cell::on_configure()
		{
			RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure of the Cell class called");
		}

		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Cell::on_activate(const rclcpp_lifecycle::State &)
		{
			std::lock_guard<std::mutex> lock(*this->mutex_);
			// set all handlers to activated
			this->active_ = true;
			RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
			for (auto &h : this->handlers_)
			{
				h.second->activate();
			}
			// call the proxy on_activate function
			this->on_activate();
			return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
		}

		void Cell::on_activate()
		{
			RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate of the Cell class called");
		}

		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Cell::on_deactivate(const rclcpp_lifecycle::State &)
		{
			RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
			std::lock_guard<std::mutex> lock(*this->mutex_);
			// set all handlers to not activated
			this->active_ = false;
			for (auto &h : this->handlers_)
			{
				h.second->deactivate();
			}
			// call the proxy on_deactivate function
			this->on_deactivate();
			return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
		}

		void Cell::on_deactivate()
		{
			RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate of the Cell class called");
		}

		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Cell::on_cleanup(const rclcpp_lifecycle::State &) 
		{
			RCUTILS_LOG_INFO_NAMED(get_name(), "on_cleanup() is called.");
			std::lock_guard<std::mutex> lock(*this->mutex_);
			this->active_ = false;
			this->running_ = false;
			// reset all handlers
			this->reset();
			// call the proxy on_cleanup function
			this->on_cleanup();
			return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
		}

		void Cell::on_cleanup()
		{
			RCUTILS_LOG_INFO_NAMED(get_name(), "on_cleanup of the Cell class called");
		}

		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Cell::on_shutdown(const rclcpp_lifecycle::State & state) 
		{
			RCUTILS_LOG_INFO_NAMED(get_name(), "on_shutdown() is called from state %s.", state.label().c_str());
			std::lock_guard<std::mutex> lock(*this->mutex_);
			this->running_ = false;
			this->active_ = false;
			this->shutdown_ = true;
			// reset all handlers for clean shutdown
			this->reset();
			// call the proxy on_shutdown function
			this->on_shutdown();
			return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
		}

		void Cell::on_shutdown()
		{
			RCUTILS_LOG_INFO_NAMED(get_name(), "on_shutdown of the Cell class called");
		}

		void Cell::run()
		{
			while(!this->shutdown_)
			{
				auto start = std::chrono::steady_clock::now();
				if (this->running_)
				{
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
				}
				auto end = std::chrono::steady_clock::now();
		    	auto elapsed = end - start;
		    	auto timeToWait = this->period_ - elapsed;
		    	if(timeToWait > std::chrono::milliseconds::zero())
		    	{
		        	std::this_thread::sleep_for(timeToWait);
		    	}
		    }
		}

		void Cell::send_transform(const StateRepresentation::CartesianPose& transform)
		{
			if (!this->running_) throw UnconfiguredNodeException("The node is not yet configured. Call the lifecycle configure before using this function");
			static_cast<Communication::TransformBroadcasterHandler&>(*this->handlers_["tf_broadcaster"]).send_transform(transform);
		}

		const StateRepresentation::CartesianPose Cell::lookup_transform(const std::string& frame_name, const std::string& reference_frame)
		{
			if (!this->running_) throw UnconfiguredNodeException("The node is not yet configured. Call the lifecycle configure before using this function");
			return static_cast<Communication::TransformListenerHandler&>(*this->handlers_["tf_listener"]).lookup_transform(frame_name, reference_frame);
		}
	}
}


