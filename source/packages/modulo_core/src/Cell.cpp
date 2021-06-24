#include "modulo_core/Cell.hpp"
#include "modulo_core/Exceptions/UnconfiguredNodeException.hpp"
#include <state_representation/exceptions/IncompatibleSizeException.hpp>
#include <state_representation/exceptions/UnrecognizedParameterTypeException.hpp>

namespace modulo::core {
Cell::~Cell() {
  this->parameters_.clear();
  this->on_shutdown();
}

void Cell::reset() {
  this->active_ = false;
  this->configured_ = false;
  this->handlers_.clear();
  for (auto& t : this->active_threads_) {
    t.join();
  }
  this->active_threads_.clear();
  this->run_thread_.join();
}

template <typename T>
void Cell::add_parameter(const std::shared_ptr<state_representation::Parameter<T>>& parameter, const std::string& prefix) {
  std::lock_guard<std::mutex> lock(*this->mutex_);
  std::string tprefix = (prefix != "") ? prefix + "_" : "";
  parameter->set_name(tprefix + parameter->get_name());
  this->parameters_.insert(std::make_pair(parameter->get_name(), parameter));
  this->declare_parameter(parameter->get_name(), parameter->get_value());
}

template void Cell::add_parameter<double>(const std::shared_ptr<state_representation::Parameter<double>>& parameter, const std::string& prefix);

template void Cell::add_parameter<std::vector<double>>(const std::shared_ptr<state_representation::Parameter<std::vector<double>>>& parameter, const std::string& prefix);

template void Cell::add_parameter<bool>(const std::shared_ptr<state_representation::Parameter<bool>>& parameter, const std::string& prefix);

template void Cell::add_parameter<std::vector<bool>>(const std::shared_ptr<state_representation::Parameter<std::vector<bool>>>& parameter, const std::string& prefix);

template void Cell::add_parameter<std::string>(const std::shared_ptr<state_representation::Parameter<std::string>>& parameter, const std::string& prefix);

template void Cell::add_parameter<std::vector<std::string>>(const std::shared_ptr<state_representation::Parameter<std::vector<std::string>>>& parameter, const std::string& prefix);

template <>
void Cell::add_parameter<state_representation::CartesianState>(const std::shared_ptr<state_representation::Parameter<state_representation::CartesianState>>& parameter, const std::string& prefix) {
  std::lock_guard<std::mutex> lock(*this->mutex_);
  std::string tprefix = (prefix != "") ? prefix + "_" : "";
  parameter->set_name(tprefix + parameter->get_name());
  this->parameters_.insert(std::make_pair(parameter->get_name(), parameter));
  this->declare_parameter<std::vector<double>>(parameter->get_name(), parameter->get_value().to_std_vector());
}

template <>
void Cell::add_parameter<state_representation::CartesianPose>(const std::shared_ptr<state_representation::Parameter<state_representation::CartesianPose>>& parameter, const std::string& prefix) {
  std::lock_guard<std::mutex> lock(*this->mutex_);
  std::string tprefix = (prefix != "") ? prefix + "_" : "";
  parameter->set_name(tprefix + parameter->get_name());
  this->parameters_.insert(std::make_pair(parameter->get_name(), parameter));
  state_representation::CartesianPose value(parameter->get_value());
  this->declare_parameter<std::vector<double>>(parameter->get_name(), value.to_std_vector());
}

template <>
void Cell::add_parameter<state_representation::JointState>(const std::shared_ptr<state_representation::Parameter<state_representation::JointState>>& parameter, const std::string& prefix) {
  std::lock_guard<std::mutex> lock(*this->mutex_);
  std::string tprefix = (prefix != "") ? prefix + "_" : "";
  parameter->set_name(tprefix + parameter->get_name());
  this->parameters_.insert(std::make_pair(parameter->get_name(), parameter));
  this->declare_parameter<std::vector<double>>(parameter->get_name(), parameter->get_value().to_std_vector());
}

template <>
void Cell::add_parameter<state_representation::JointPositions>(const std::shared_ptr<state_representation::Parameter<state_representation::JointPositions>>& parameter, const std::string& prefix) {
  std::lock_guard<std::mutex> lock(*this->mutex_);
  std::string tprefix = (prefix != "") ? prefix + "_" : "";
  parameter->set_name(tprefix + parameter->get_name());
  this->parameters_.insert(std::make_pair(parameter->get_name(), parameter));
  state_representation::JointPositions value(parameter->get_value());
  this->declare_parameter<std::vector<double>>(parameter->get_name(), value.to_std_vector());
}

template <>
void Cell::add_parameter<state_representation::Ellipsoid>(const std::shared_ptr<state_representation::Parameter<state_representation::Ellipsoid>>& parameter, const std::string& prefix) {
  std::lock_guard<std::mutex> lock(*this->mutex_);
  std::string tprefix = (prefix != "") ? prefix + "_" : "";
  parameter->set_name(tprefix + parameter->get_name());
  this->parameters_.insert(std::make_pair(parameter->get_name(), parameter));
  this->declare_parameter<std::vector<double>>(parameter->get_name(), parameter->get_value().to_std_vector());
}

template <>
void Cell::add_parameter<Eigen::MatrixXd>(const std::shared_ptr<state_representation::Parameter<Eigen::MatrixXd>>& parameter, const std::string& prefix) {
  std::lock_guard<std::mutex> lock(*this->mutex_);
  std::string tprefix = (prefix != "") ? prefix + "_" : "";
  parameter->set_name(tprefix + parameter->get_name());
  this->parameters_.insert(std::make_pair(parameter->get_name(), parameter));
  std::vector<double> value = std::vector<double>(parameter->get_value().data(), parameter->get_value().data() + parameter->get_value().size());
  this->declare_parameter<std::vector<double>>(parameter->get_name(), value);
}

void Cell::add_parameters(const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters, const std::string& prefix) {
  using namespace state_representation;
  using namespace state_representation::exceptions;
  for (auto& param : parameters) {
    switch (param->get_type()) {
      case StateType::PARAMETER_DOUBLE: {
        this->add_parameter(std::static_pointer_cast<Parameter<double>>(param), prefix);
        break;
      }

      case StateType::PARAMETER_DOUBLE_ARRAY: {
        this->add_parameter(std::static_pointer_cast<Parameter<std::vector<double>>>(param), prefix);
        break;
      }

      case StateType::PARAMETER_BOOL: {
        this->add_parameter(std::static_pointer_cast<Parameter<bool>>(param), prefix);
        break;
      }

      case StateType::PARAMETER_BOOL_ARRAY: {
        this->add_parameter(std::static_pointer_cast<Parameter<std::vector<double>>>(param), prefix);
        break;
      }

      case StateType::PARAMETER_STRING: {
        this->add_parameter(std::static_pointer_cast<Parameter<std::string>>(param), prefix);
        break;
      }

      case StateType::PARAMETER_STRING_ARRAY: {
        this->add_parameter(std::static_pointer_cast<Parameter<std::vector<std::string>>>(param), prefix);
        break;
      }

      case StateType::PARAMETER_CARTESIANSTATE: {
        this->add_parameter(std::static_pointer_cast<Parameter<CartesianState>>(param), prefix);
        break;
      }

      case StateType::PARAMETER_CARTESIANPOSE: {
        this->add_parameter(std::static_pointer_cast<Parameter<CartesianPose>>(param), prefix);
        break;
      }

      case StateType::PARAMETER_JOINTSTATE: {
        this->add_parameter(std::static_pointer_cast<Parameter<JointState>>(param), prefix);
        break;
      }

      case StateType::PARAMETER_JOINTPOSITIONS: {
        this->add_parameter(std::static_pointer_cast<Parameter<JointPositions>>(param), prefix);
        break;
      }

      case StateType::PARAMETER_ELLIPSOID: {
        this->add_parameter(std::static_pointer_cast<Parameter<Ellipsoid>>(param), prefix);
        break;
      }

      case StateType::PARAMETER_MATRIX: {
        this->add_parameter(std::static_pointer_cast<Parameter<Eigen::MatrixXd>>(param), prefix);
        break;
      }

      default: {
        throw UnrecognizedParameterTypeException("The Parameter type is not available");
      }
    }
  }
}

template <typename T>
void Cell::set_parameter_value(const std::string& parameter_name, const T& value) {
  this->set_parameter(rclcpp::Parameter(parameter_name, value));
}

template void Cell::set_parameter_value(const std::string& parameter_name, const double& value);

template void Cell::set_parameter_value(const std::string& parameter_name, const std::vector<double>& value);

template void Cell::set_parameter_value(const std::string& parameter_name, const bool& value);

template void Cell::set_parameter_value(const std::string& parameter_name, const std::vector<bool>& value);

template void Cell::set_parameter_value(const std::string& parameter_name, const char* const& value);

template void Cell::set_parameter_value(const std::string& parameter_name, const std::string& value);

template void Cell::set_parameter_value(const std::string& parameter_name, const std::vector<std::string>& value);

template <>
void Cell::set_parameter_value(const std::string& parameter_name, const state_representation::CartesianState& value) {
  std::vector<double> vector_value = value.to_std_vector();
  this->set_parameter_value<std::vector<double>>(parameter_name, vector_value);
}

template <>
void Cell::set_parameter_value(const std::string& parameter_name, const state_representation::CartesianPose& value) {
  std::vector<double> vector_value = value.to_std_vector();
  this->set_parameter_value<std::vector<double>>(parameter_name, vector_value);
}

template <>
void Cell::set_parameter_value(const std::string& parameter_name, const state_representation::JointState& value) {
  std::vector<double> vector_value = value.to_std_vector();
  this->set_parameter_value<std::vector<double>>(parameter_name, vector_value);
}

template <>
void Cell::set_parameter_value(const std::string& parameter_name, const state_representation::JointPositions& value) {
  std::vector<double> vector_value = value.to_std_vector();
  this->set_parameter_value<std::vector<double>>(parameter_name, vector_value);
}

template <>
void Cell::set_parameter_value(const std::string& parameter_name, const state_representation::Ellipsoid& value) {
  std::vector<double> vector_value = value.to_std_vector();
  this->set_parameter_value<std::vector<double>>(parameter_name, vector_value);
}

template <>
void Cell::set_parameter_value(const std::string& parameter_name, const Eigen::MatrixXd& value) {
  std::vector<double> vector_value = std::vector<double>(value.data(), value.data() + value.size());
  this->set_parameter_value<std::vector<double>>(parameter_name, vector_value);
}

void Cell::set_parameter_value(const std::shared_ptr<state_representation::ParameterInterface>& parameter) {
  using namespace state_representation;
  using namespace state_representation::exceptions;
  switch (parameter->get_type()) {
    case StateType::PARAMETER_DOUBLE: {
      this->set_parameter_value(std::static_pointer_cast<Parameter<double>>(parameter));
      break;
    }

    case StateType::PARAMETER_DOUBLE_ARRAY: {
      this->set_parameter_value(std::static_pointer_cast<Parameter<std::vector<double>>>(parameter));
      break;
    }

    case StateType::PARAMETER_BOOL: {
      this->set_parameter_value(std::static_pointer_cast<Parameter<bool>>(parameter));
      break;
    }

    case StateType::PARAMETER_BOOL_ARRAY: {
      this->set_parameter_value(std::static_pointer_cast<Parameter<std::vector<double>>>(parameter));
      break;
    }

    case StateType::PARAMETER_STRING: {
      this->set_parameter_value(std::static_pointer_cast<Parameter<std::string>>(parameter));
      break;
    }

    case StateType::PARAMETER_STRING_ARRAY: {
      this->set_parameter_value(std::static_pointer_cast<Parameter<std::vector<std::string>>>(parameter));
      break;
    }

    case StateType::PARAMETER_CARTESIANSTATE: {
      this->set_parameter_value(std::static_pointer_cast<Parameter<CartesianState>>(parameter));
      break;
    }

    case StateType::PARAMETER_CARTESIANPOSE: {
      this->set_parameter_value(std::static_pointer_cast<Parameter<CartesianPose>>(parameter));
      break;
    }

    case StateType::PARAMETER_JOINTSTATE: {
      this->set_parameter_value(std::static_pointer_cast<Parameter<JointState>>(parameter));
      break;
    }

    case StateType::PARAMETER_JOINTPOSITIONS: {
      this->set_parameter_value(std::static_pointer_cast<Parameter<JointPositions>>(parameter));
      break;
    }

    case StateType::PARAMETER_ELLIPSOID: {
      this->set_parameter_value(std::static_pointer_cast<Parameter<Ellipsoid>>(parameter));
      break;
    }

    case StateType::PARAMETER_MATRIX: {
      this->set_parameter_value(std::static_pointer_cast<Parameter<Eigen::MatrixXd>>(parameter));
      break;
    }

    default: {
      throw UnrecognizedParameterTypeException("The Parameter type is not available");
    }
  }
}

void Cell::add_transform_broadcaster(const std::shared_ptr<state_representation::CartesianState>& recipient,
                                     bool always_active,
                                     int queue_size) {
  this->add_transform_broadcaster(recipient, this->get_period(), always_active, queue_size);
}

void Cell::add_transform_broadcaster(const state_representation::CartesianPose& recipient,
                                     bool always_active,
                                     int queue_size) {
  this->add_transform_broadcaster(recipient, this->get_period(), always_active, queue_size);
}

void Cell::add_transform_broadcaster(const std::shared_ptr<state_representation::ParameterInterface>& recipient,
                                     bool always_active,
                                     int queue_size) {
  this->add_transform_broadcaster(recipient, this->get_period(), always_active, queue_size);
}

void Cell::send_transform(const state_representation::CartesianState& transform, const std::string& name) const {
  if (!this->configured_) throw exceptions::UnconfiguredNodeException("The node is not yet configured. Call the lifecycle configure before using this function");
  state_representation::CartesianState transform_copy(transform);
  if (name != "") transform_copy.set_name(name);
  std::static_pointer_cast<communication::TransformBroadcasterHandler>(this->handlers_.at("tf_broadcaster").first)->send_transform(transform_copy);
}

const state_representation::CartesianPose Cell::lookup_transform(const std::string& frame_name, const std::string& reference_frame) const {
  if (!this->configured_) throw exceptions::UnconfiguredNodeException("The node is not yet configured. Call the lifecycle configure before using this function");
  return std::static_pointer_cast<communication::TransformListenerHandler>(this->handlers_.at("tf_listener").first)->lookup_transform(frame_name, reference_frame);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Cell::on_configure(const rclcpp_lifecycle::State&) {
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");
  this->active_ = false;
  this->configured_ = true;
  // call the proxy on_configure function
  if (!this->on_configure()) {
    RCLCPP_ERROR(get_logger(), "Configuration failed");
    this->reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  // add the run thread
  std::function<void(void)> run_fnc = std::bind(&Cell::run, this);
  this->run_thread_ = std::thread(run_fnc);
  // add default transform broadcaster and transform listener
  this->add_transform_broadcaster(this->period_, true);
  this->add_transform_listener(10 * this->period_);
  // set all always active handlers to be activated
  for (auto& h : this->handlers_) {
    if (h.second.second) h.second.first->activate();
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool Cell::on_configure() {
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure of the Cell class called");
  return true;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Cell::on_activate(const rclcpp_lifecycle::State&) {
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
  // call the proxy on_activate function
  if (!this->on_activate()) {
    RCLCPP_ERROR(get_logger(), "Activation failed.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  // set all handlers to activated
  this->active_ = true;
  for (auto& h : this->handlers_) {
    h.second.first->activate();
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool Cell::on_activate() {
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate of the Cell class called");
  return true;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Cell::on_deactivate(const rclcpp_lifecycle::State&) {
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
  // call the proxy on_deactivate function
  if (!this->on_deactivate()) {
    RCLCPP_ERROR(get_logger(), "Deactivation failed.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  // set all handlers to not activated
  this->active_ = false;
  for (auto& h : this->handlers_) {
    if (!h.second.second) h.second.first->deactivate();
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool Cell::on_deactivate() {
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate of the Cell class called");
  return true;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Cell::on_cleanup(const rclcpp_lifecycle::State&) {
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_cleanup() is called.");
  // call the proxy on_cleanup function
  if (!this->on_cleanup()) {
    RCLCPP_ERROR(get_logger(), "Cleanup failed.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  // reset all handlers
  this->reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool Cell::on_cleanup() {
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_cleanup of the Cell class called");
  return true;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Cell::on_shutdown(const rclcpp_lifecycle::State& state) {
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_shutdown() is called from state %s.", state.label().c_str());
  // call the proxy on_shutdown function
  if (!this->on_shutdown()) {
    RCLCPP_ERROR(get_logger(), "Shutdown failed.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  // reset all handlers for clean shutdown
  this->reset();
  this->shutdown_ = true;
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool Cell::on_shutdown() {
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_shutdown of the Cell class called");
  return true;
}

void Cell::run() {
  while (this->configured_) {
    auto start = std::chrono::steady_clock::now();
    std::unique_lock<std::mutex> lck(*this->mutex_);
    for (auto& h : this->handlers_) {
      h.second.first->check_timeout();
    }
    if (this->active_) {
      this->step();
    }
    lck.unlock();
    auto end = std::chrono::steady_clock::now();
    auto elapsed = end - start;
    auto timeToWait = this->period_ - elapsed;
    if (timeToWait > std::chrono::nanoseconds::zero()) {
      std::this_thread::sleep_for(timeToWait);
    }
  }
}

void Cell::step() {}

void Cell::run_periodic_call(const std::function<void(void)>& callback_function, const std::chrono::nanoseconds& period) {
  while (this->configured_) {
    auto start = std::chrono::steady_clock::now();
    std::unique_lock<std::mutex> lck(*this->mutex_);
    if (this->active_) {
      callback_function();
    }
    lck.unlock();
    auto end = std::chrono::steady_clock::now();
    auto elapsed = end - start;
    auto timeToWait = period - elapsed;
    if (timeToWait > std::chrono::nanoseconds::zero()) {
      std::this_thread::sleep_for(timeToWait);
    }
  }
}

void Cell::add_periodic_call(const std::function<void(void)>& callback_function, const std::chrono::nanoseconds& period) {
  std::function<void(const std::function<void(void)>&, const std::chrono::nanoseconds&)> fnc = std::bind(&Cell::run_periodic_call, this, callback_function, period);
  this->active_threads_.push_back(std::thread(fnc, callback_function, period));
}

void Cell::update_parameters() {
  using namespace state_representation;
  using namespace state_representation::exceptions;
  while (!this->shutdown_) {
    auto start = std::chrono::steady_clock::now();
    try {
      for (auto& [key, param] : this->parameters_) {
        switch (param->get_type()) {
          case StateType::PARAMETER_DOUBLE: {
            std::unique_lock<std::mutex> lck(*this->mutex_);
            double value = this->get_parameter(param->get_name()).as_double();
            std::static_pointer_cast<Parameter<double>>(param)->set_value(value);
            lck.unlock();
            break;
          }

          case StateType::PARAMETER_DOUBLE_ARRAY: {
            std::unique_lock<std::mutex> lck(*this->mutex_);
            std::vector<double> value = this->get_parameter(param->get_name()).as_double_array();
            std::static_pointer_cast<Parameter<std::vector<double>>>(param)->set_value(value);
            lck.unlock();
            break;
          }

          case StateType::PARAMETER_BOOL: {
            std::unique_lock<std::mutex> lck(*this->mutex_);
            bool value = this->get_parameter(param->get_name()).as_bool();
            std::static_pointer_cast<Parameter<bool>>(param)->set_value(value);
            lck.unlock();
            break;
          }

          case StateType::PARAMETER_BOOL_ARRAY: {
            std::unique_lock<std::mutex> lck(*this->mutex_);
            std::vector<bool> value = this->get_parameter(param->get_name()).as_bool_array();
            std::static_pointer_cast<Parameter<std::vector<bool>>>(param)->set_value(value);
            lck.unlock();
            break;
          }

          case StateType::PARAMETER_STRING: {
            std::unique_lock<std::mutex> lck(*this->mutex_);
            std::string value = this->get_parameter(param->get_name()).as_string();
            std::static_pointer_cast<Parameter<std::string>>(param)->set_value(value);
            lck.unlock();
            break;
          }

          case StateType::PARAMETER_STRING_ARRAY: {
            std::unique_lock<std::mutex> lck(*this->mutex_);
            std::vector<std::string> value = this->get_parameter(param->get_name()).as_string_array();
            std::static_pointer_cast<Parameter<std::vector<std::string>>>(param)->set_value(value);
            lck.unlock();
            break;
          }

          case StateType::PARAMETER_CARTESIANSTATE: {
            std::unique_lock<std::mutex> lck(*this->mutex_);
            std::vector<double> value = this->get_parameter(param->get_name()).as_double_array();
            std::static_pointer_cast<Parameter<CartesianState>>(param)->get_value().from_std_vector(value);
            lck.unlock();
            break;
          }

          case StateType::PARAMETER_CARTESIANPOSE: {
            std::unique_lock<std::mutex> lck(*this->mutex_);
            std::vector<double> value = this->get_parameter(param->get_name()).as_double_array();
            std::static_pointer_cast<Parameter<CartesianPose>>(param)->get_value().CartesianPose::from_std_vector(value);
            lck.unlock();
            break;
          }

          case StateType::PARAMETER_JOINTSTATE: {
            std::unique_lock<std::mutex> lck(*this->mutex_);
            std::vector<double> value = this->get_parameter(param->get_name()).as_double_array();
            std::static_pointer_cast<Parameter<JointState>>(param)->get_value().from_std_vector(value);
            lck.unlock();
            break;
          }

          case StateType::PARAMETER_JOINTPOSITIONS: {
            std::unique_lock<std::mutex> lck(*this->mutex_);
            std::vector<double> value = this->get_parameter(param->get_name()).as_double_array();
            std::static_pointer_cast<Parameter<JointPositions>>(param)->get_value().JointPositions::from_std_vector(value);
            lck.unlock();
            break;
          }

          case StateType::PARAMETER_ELLIPSOID: {
            std::unique_lock<std::mutex> lck(*this->mutex_);
            std::vector<double> value = this->get_parameter(param->get_name()).as_double_array();
            std::static_pointer_cast<Parameter<Ellipsoid>>(param)->get_value().from_std_vector(value);
            lck.unlock();
            break;
          }

          case StateType::PARAMETER_MATRIX: {
            std::unique_lock<std::mutex> lck(*this->mutex_);
            std::vector<double> value = this->get_parameter(param->get_name()).as_double_array();
            size_t rows = std::static_pointer_cast<Parameter<Eigen::MatrixXd>>(param)->get_value().rows();
            size_t cols = std::static_pointer_cast<Parameter<Eigen::MatrixXd>>(param)->get_value().cols();
            size_t size = std::static_pointer_cast<Parameter<Eigen::MatrixXd>>(param)->get_value().size();
            // depending on the size of the received parameter produce a different matrix
            Eigen::MatrixXd matrix_value(rows, cols);
            if (value.size() == size)// equal size direct copy
            {
              matrix_value = Eigen::MatrixXd::Map(value.data(), rows, cols);
            } else if (value.size() == rows && value.size() == cols)// diagonal matrix with only diagonal values set
            {
              Eigen::VectorXd diagonal_coefficients = Eigen::VectorXd::Map(value.data(), value.size());
              matrix_value = diagonal_coefficients.asDiagonal();
            } else if (value.size() == 1)// single element means iso diagonal matrix
            {
              matrix_value = value[0] * Eigen::MatrixXd::Identity(rows, cols);
            } else// any other sizes generates an error
            {
              throw IncompatibleSizeException("The value set does not have the correct expected size of " + std::to_string(rows) + "x" + std::to_string(cols) + "elements");
            }
            std::static_pointer_cast<Parameter<Eigen::MatrixXd>>(param)->set_value(matrix_value);
            lck.unlock();
            break;
          }

          default: {
            throw UnrecognizedParameterTypeException("The Parameter type is not available");
          }
        }
      }
    } catch (rclcpp::ParameterTypeException& e) {
      RCLCPP_ERROR(this->get_logger(), e.what());
    } catch (IncompatibleSizeException& e) {
      RCLCPP_ERROR(this->get_logger(), e.what());
    } catch (UnrecognizedParameterTypeException& e) {
      RCLCPP_ERROR(this->get_logger(), e.what());
    }
    auto end = std::chrono::steady_clock::now();
    auto elapsed = end - start;
    auto timeToWait = this->period_ - elapsed;
    if (timeToWait > std::chrono::nanoseconds::zero()) {
      std::this_thread::sleep_for(timeToWait);
    }
  }
}
}// namespace modulo::core
