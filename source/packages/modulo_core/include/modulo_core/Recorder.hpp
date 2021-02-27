#pragma once

#include "modulo_core/Cell.hpp"
#include <typeinfo>

namespace modulo::core {
/**
 * @class Recorder
 * @brief Abstract class to define a Recorder
 * A Recorder is used to listen on specific topics and record
 * the received data.
 */
class Recorder : public Cell {
private:
  std::chrono::time_point<std::chrono::system_clock> start_time;///< start time of current recording session, reset on activate
  std::chrono::time_point<std::chrono::system_clock> end_time;  ///< end time of current recording session, reset on deactivate

public:
  /**
   * @brief Constructor for the Recorder class
   * @param node_name name of the ROS node
   * @param period rate used by each publisher of the class
   */
  template <typename DurationT>
  explicit Recorder(const std::string& node_name, const std::chrono::duration<int64_t, DurationT>& period, bool intra_process_comms = false);

  /**
   * @brief Destructor
   */
  ~Recorder();

  /**
   * @brief Getter of the start_time attribute
   * @return start_time
   */
  const auto& get_start_time() const;

  /**
   * @brief Getter of the end_time attribute
   * @return end_time
   */
  const auto& get_end_time() const;

  /**
   * @brief This function is called time the configure call 
   * is made from the lifecycle server. It is used to
   * define behavior such as connecting to a database 
   * or resetting an history buffer. After being 
   * configured the node can be activated.
   */
  virtual bool on_configure();

  /**
   * @brief This function is called time the activate call 
   * is made from the lifecycle server. It activates publishing
   * and subsciptions and can be extended to start a recording
   * or replay.
   */
  bool on_activate();

  /**
   * @brief This function is called time the deactivate call 
   * is made from the lifecycle server. It deactivates publishing
   * and subsciptions and can be extended to stop a recording
   * or a replay.
   */
  bool on_deactivate();

  /**
   * @brief This function is called time the cleanup call 
   * is made from the lifecycle server. It cleans the node
   * and can be extended to close connections to a database
   * or delete pointers. After cleanup a new configure call
   * can be made.
   */
  virtual bool on_cleanup();

  /**
   * @brief This function is called time the shutdown call 
   * is made from the lifecycle server. It terminates the node.
   * Each elements needed to be cleaned before termination should
   * be here.
   */
  virtual bool on_shutdown();

  /**
   * @brief Function computing one step of calculation. It is called periodically in the run function.
   */
  void step();

  /**
   * @brief Generic function to record a state. This function is called in the step
   * function for each subscription the recorder has
   * @param state the state to be recorded
   * @return true if the state was successfully recorded
   */
  bool record(const StateRepresentation::State& state) const;

  /**
   * @brief Abstract function to record a CartesianState. This function needs to be
   * implemeted in the derived class.
   * @param state the state to be recorded
   * @return true if the state was successfully recorded
   */
  virtual bool record(const StateRepresentation::CartesianState& state) const;

  /**
   * @brief Abstract function to record a JointState. This function needs to be
   * implemeted in the derived class.
   * @param state the state to be recorded
   * @return true if the state was successfully recorded
   */
  virtual bool record(const StateRepresentation::JointState& state) const;
};

template <typename DurationT>
Recorder::Recorder(const std::string& node_name, const std::chrono::duration<int64_t, DurationT>& period, bool intra_process_comms) : Cell(node_name, period, intra_process_comms) {}

inline const auto& Recorder::get_start_time() const {
  return start_time;
}

inline const auto& Recorder::get_end_time() const {
  return end_time;
}
}// namespace modulo::core
