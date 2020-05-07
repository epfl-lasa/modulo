#include "modulo_core/Cell.hpp"
#include "modulo_core/Exceptions/UnconfiguredNodeException.hpp"
#include "state_representation/Exceptions/UnrecognizedParameterTypeException.hpp"
#include "state_representation/Exceptions/IncompatibleSizeException.hpp"

namespace Modulo
{
	namespace Core
	{
		Cell::~Cell()
		{
			this->parameters_.clear();
			this->on_shutdown();
		}

		void Cell::reset()
		{
			this->active_ = false;
			this->configured_ = false;
			this->handlers_.clear();
			for(auto& t:this->active_threads_)
			{
				t.join();
			}
			this->active_threads_.clear();
			this->run_thread_.join();
		}

		template <typename T>
		void Cell::add_parameter(const std::shared_ptr<StateRepresentation::Parameter<T>>& parameter, const std::string& prefix)
		{
			std::string tprefix = (prefix != "") ? prefix + "_" : "";
			parameter->set_name(tprefix + parameter->get_name());
			this->parameters_.push_back(parameter);
			this->declare_parameter(parameter->get_name(), parameter->get_value());
		}

		template void Cell::add_parameter<double>(const std::shared_ptr<StateRepresentation::Parameter<double>>& parameter, const std::string& prefix);

		template void Cell::add_parameter<std::vector<double>>(const std::shared_ptr<StateRepresentation::Parameter<std::vector<double>>>& parameter, const std::string& prefix);

		template void Cell::add_parameter<bool>(const std::shared_ptr<StateRepresentation::Parameter<bool>>& parameter, const std::string& prefix);

		template void Cell::add_parameter<std::vector<bool>>(const std::shared_ptr<StateRepresentation::Parameter<std::vector<bool>>>& parameter, const std::string& prefix);

		template void Cell::add_parameter<std::string>(const std::shared_ptr<StateRepresentation::Parameter<std::string>>& parameter, const std::string& prefix);

		template void Cell::add_parameter<std::vector<std::string>>(const std::shared_ptr<StateRepresentation::Parameter<std::vector<std::string>>>& parameter, const std::string& prefix);

		template <>
		void Cell::add_parameter<StateRepresentation::CartesianState>(const std::shared_ptr<StateRepresentation::Parameter<StateRepresentation::CartesianState>>& parameter, const std::string& prefix)
		{
			std::string tprefix = (prefix != "") ? prefix + "_" : "";
			parameter->set_name(tprefix + parameter->get_name());
			this->parameters_.push_back(parameter);
			this->declare_parameter<std::vector<double>>(parameter->get_name(), parameter->get_value().StateRepresentation::CartesianState::to_std_vector());
		}

		template <>
		void Cell::add_parameter<StateRepresentation::CartesianPose>(const std::shared_ptr<StateRepresentation::Parameter<StateRepresentation::CartesianPose>>& parameter, const std::string& prefix)
		{
			std::string tprefix = (prefix != "") ? prefix + "_" : "";
			parameter->set_name(tprefix + parameter->get_name());
			this->parameters_.push_back(parameter);
			this->declare_parameter<std::vector<double>>(parameter->get_name(), parameter->get_value().StateRepresentation::CartesianPose::to_std_vector());
		}

		template<>
		void Cell::add_parameter<StateRepresentation::JointState>(const std::shared_ptr<StateRepresentation::Parameter<StateRepresentation::JointState>>& parameter, const std::string& prefix)
		{
			std::string tprefix = (prefix != "") ? prefix + "_" : "";
			parameter->set_name(tprefix + parameter->get_name());
			this->parameters_.push_back(parameter);
			this->declare_parameter<std::vector<double>>(parameter->get_name(), parameter->get_value().StateRepresentation::JointState::to_std_vector());
		}

		template<>
		void Cell::add_parameter<StateRepresentation::JointPositions>(const std::shared_ptr<StateRepresentation::Parameter<StateRepresentation::JointPositions>>& parameter, const std::string& prefix)
		{
			std::string tprefix = (prefix != "") ? prefix + "_" : "";
			parameter->set_name(tprefix + parameter->get_name());
			this->parameters_.push_back(parameter);
			this->declare_parameter<std::vector<double>>(parameter->get_name(), parameter->get_value().StateRepresentation::JointPositions::to_std_vector());
		}

		void Cell::add_parameters(const std::list<std::shared_ptr<StateRepresentation::ParameterInterface>>& parameters, const std::string& prefix)
		{
			using namespace StateRepresentation;
			using namespace StateRepresentation::Exceptions;
			for (auto& param : parameters)
			{
				switch (param->get_type())
				{
					case StateType::PARAMETER_DOUBLE:
					{
						this->add_parameter(std::static_pointer_cast<Parameter<double>>(param), prefix);
						break;
					}

					case StateType::PARAMETER_DOUBLE_ARRAY:
					{
						this->add_parameter(std::static_pointer_cast<Parameter<std::vector<double>>>(param), prefix);
						break;
					}

					case StateType::PARAMETER_BOOL:
					{
						this->add_parameter(std::static_pointer_cast<Parameter<bool>>(param), prefix);
						break;
					}

					case StateType::PARAMETER_BOOL_ARRAY:
					{
						this->add_parameter(std::static_pointer_cast<Parameter<std::vector<double>>>(param), prefix);
						break;
					}

					case StateType::PARAMETER_STRING:
					{
						this->add_parameter(std::static_pointer_cast<Parameter<std::string>>(param), prefix);
						break;
					}

					case StateType::PARAMETER_STRING_ARRAY:
					{
						this->add_parameter(std::static_pointer_cast<Parameter<std::vector<std::string>>>(param), prefix);
						break;
					}

					case StateType::PARAMETER_CARTESIANSTATE:
					{
						this->add_parameter(std::static_pointer_cast<Parameter<CartesianState>>(param), prefix);
						break;
					}

					case StateType::PARAMETER_CARTESIANPOSE:
					{
						this->add_parameter(std::static_pointer_cast<Parameter<CartesianPose>>(param), prefix);
						break;
					}

					case StateType::PARAMETER_JOINTSTATE:
					{
						this->add_parameter(std::static_pointer_cast<Parameter<JointState>>(param), prefix);
						break;
					}

					case StateType::PARAMETER_JOINTPOSITIONS:
					{
						this->add_parameter(std::static_pointer_cast<Parameter<JointPositions>>(param), prefix);
						break;
					}

					default:
					{
						throw UnrecognizedParameterTypeException("The Parameter type is not available");
						break;
					}
				}
			}
		}

		void Cell::send_transform(const StateRepresentation::CartesianState& transform)
		{
			if (!this->configured_) throw Exceptions::UnconfiguredNodeException("The node is not yet configured. Call the lifecycle configure before using this function");
			static_cast<Communication::MessagePassing::TransformBroadcasterHandler&>(*this->handlers_["tf_broadcaster"]).send_transform(transform);
		}

		const StateRepresentation::CartesianPose Cell::lookup_transform(const std::string& frame_name, const std::string& reference_frame)
		{
			if (!this->configured_) throw Exceptions::UnconfiguredNodeException("The node is not yet configured. Call the lifecycle configure before using this function");
			return static_cast<Communication::MessagePassing::TransformListenerHandler&>(*this->handlers_["tf_listener"]).lookup_transform(frame_name, reference_frame);
		}

		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Cell::on_configure(const rclcpp_lifecycle::State &) 
		{
			RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");
			// call the proxy on_configure function
			if(!this->on_configure())
			{
				RCLCPP_ERROR(get_logger(), "Configuration failed");
				this->reset();
				return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
			}
			// start the parameters
			this->active_ = false;
			this->configured_ = true;
			// add the run thread
			std::function<void(void)> run_fnc = std::bind(&Cell::run, this);
			this->run_thread_ = std::thread(run_fnc);
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

		void Cell::run_periodic_call(const std::function<void(void)>& callback_function, const std::chrono::nanoseconds& period)
		{
			while(this->configured_)
			{
				auto start = std::chrono::steady_clock::now();
				std::unique_lock<std::mutex> lck(*this->mutex_);
				if(this->active_)
				{
					callback_function();
				}
				lck.unlock();
				auto end = std::chrono::steady_clock::now();
		    	auto elapsed = end - start;
		    	auto timeToWait = period - elapsed;
		    	if(timeToWait > std::chrono::nanoseconds::zero())
		    	{
		        	std::this_thread::sleep_for(timeToWait);
		    	}
			}
		}

		void Cell::add_periodic_call(const std::function<void(void)>& callback_function, const std::chrono::nanoseconds& period)
		{
			std::function<void(const std::function<void(void)>&, const std::chrono::nanoseconds&)> fnc = std::bind(&Cell::run_periodic_call, this, callback_function, period);
			this->active_threads_.push_back(std::thread(fnc, callback_function, period));
		}

		void Cell::update_parameters()
		{
			using namespace StateRepresentation;
			using namespace StateRepresentation::Exceptions;
			while(!this->shutdown_)
			{
				auto start = std::chrono::steady_clock::now();
				try
				{
					for (auto& param : this->parameters_)
					{
						switch (param->get_type())
						{
							case StateType::PARAMETER_DOUBLE:
							{
								std::unique_lock<std::mutex> lck(*this->mutex_);
								double value = this->get_parameter(param->get_name()).as_double();
								static_cast<Parameter<double>&>(*param).set_value(value);
								lck.unlock();
								break;
							}

							case StateType::PARAMETER_DOUBLE_ARRAY:
							{
								std::unique_lock<std::mutex> lck(*this->mutex_);
								std::vector<double> value = this->get_parameter(param->get_name()).as_double_array();
								static_cast<Parameter<std::vector<double>>&>(*param).set_value(value);
								lck.unlock();
								break;
							}

							case StateType::PARAMETER_BOOL:
							{
								std::unique_lock<std::mutex> lck(*this->mutex_);
								bool value = this->get_parameter(param->get_name()).as_bool();
								static_cast<Parameter<bool>&>(*param).set_value(value);
								lck.unlock();
								break;
							}

							case StateType::PARAMETER_BOOL_ARRAY:
							{
								std::unique_lock<std::mutex> lck(*this->mutex_);
								std::vector<bool> value = this->get_parameter(param->get_name()).as_bool_array();
								static_cast<Parameter<std::vector<bool>>&>(*param).set_value(value);
								lck.unlock();
								break;
							}

							case StateType::PARAMETER_STRING:
							{
								std::unique_lock<std::mutex> lck(*this->mutex_);
								std::string value = this->get_parameter(param->get_name()).as_string();
								static_cast<Parameter<std::string>&>(*param).set_value(value);
								lck.unlock();
								break;
							}

							case StateType::PARAMETER_STRING_ARRAY:
							{
								std::unique_lock<std::mutex> lck(*this->mutex_);
								std::vector<std::string> value = this->get_parameter(param->get_name()).as_string_array();
								static_cast<Parameter<std::vector<std::string>>&>(*param).set_value(value);
								lck.unlock();
								break;
							}

							case StateType::PARAMETER_CARTESIANSTATE:
							{
								std::unique_lock<std::mutex> lck(*this->mutex_);
								std::vector<double> value = this->get_parameter(param->get_name()).as_double_array();
								static_cast<Parameter<CartesianState>&>(*param).get_value().CartesianState::from_std_vector(value);
								lck.unlock();
								break;
							}

							case StateType::PARAMETER_CARTESIANPOSE:
							{
								std::unique_lock<std::mutex> lck(*this->mutex_);
								std::vector<double> value = this->get_parameter(param->get_name()).as_double_array();
								static_cast<Parameter<CartesianPose>&>(*param).get_value().CartesianPose::from_std_vector(value);
								lck.unlock();
								break;
							}

							/*case StateType::PARAMETER_JOINTPOSITIONS:
							{
								std::unique_lock<std::mutex> lck(*this->mutex_);
								std::vector<double> value = this->get_parameter(param->get_name()).as_double_array();
								static_cast<Parameter<JointPositions>&>(*param).set_value(value);
								lck.unlock();
								break;
							}*/

							default:
							{
								throw UnrecognizedParameterTypeException("The Parameter type is not available");
								break;
							}
						}
					}
				}
				catch (rclcpp::ParameterTypeException& e)
				{
					RCLCPP_ERROR(this->get_logger(), e.what());
				}
				catch (IncompatibleSizeException& e)
				{
					RCLCPP_ERROR(this->get_logger(), e.what());
				}
				catch (UnrecognizedParameterTypeException& e)
				{
					RCLCPP_ERROR(this->get_logger(), e.what());
				}
				auto end = std::chrono::steady_clock::now();
		    	auto elapsed = end - start;
		    	auto timeToWait = this->period_ - elapsed;
		    	if(timeToWait > std::chrono::nanoseconds::zero())
		    	{
		        	std::this_thread::sleep_for(timeToWait);
		    	}
			}
		}
	}
}


