#pragma once

#include <iostream>
#include <mutex>
#include <thread>
#include <chrono>

#include <communication_interfaces/sockets/ZMQPublisherSubscriber.hpp>
#include <state_representation/space/joint/JointState.hpp>

#include <DRFLEx.h>

namespace doosanlwi {

class DoosanLightWeightInterface {
private:
  std::string prefix_; ///< prefix of the robot joints
  std::string robot_ip_; ///< ip of the robot to connect to
  DRAFramework::CDRFLEx drfl_; ///< robot object to send command to
  bool connected_;
  bool shutdown_;
  std::shared_ptr<communication_interfaces::sockets::ZMQPublisherSubscriber> sockets_;
  state_representation::JointState state_;
  state_representation::JointState command_;
  state_representation::JointStateVariable control_type_;
  std::chrono::steady_clock::time_point last_command_;
  std::chrono::milliseconds command_timeout_ = std::chrono::milliseconds(500);
  std::mutex mutex_;

  void print_state() const;

public:
  typedef void (DoosanLightWeightInterface::*cb)(const ROBOT_STATE);

  explicit DoosanLightWeightInterface(
      std::string robot_ip, communication_interfaces::sockets::ZMQCombinedSocketsConfiguration sockets_configuration,
      std::string prefix
  );

  /**
   * @brief Getter of the connected boolean attribute
   * @return the value of the connected attribute
   */
  bool is_connected() const;

  /**
   * @brief Getter of the shutdown boolean attribute
   * @return the value of the shutdown attribute
   */
  bool is_shutdown() const;

  /**
   * Getter of the mutex attribute
   * @return the mutex attribute
   */
  std::mutex& get_mutex();

  /**
   * @brief Initialize the connection to the robot
   */
  void init();

  /**
   * @brief Reset the commanded state variable derivatives to zero (twists, accelerations and torques).
   */
  void reset_command();

  /**
   * @brief Threaded function that run a controller based on the value in the active_controller enumeration
   */
  void run_controller();

  /**
   * @brief Poll the ZMQ socket subscription for a new joint torque command from an external controller
   */
  void poll_external_command();

  void publish_robot_state();

  void read_robot_state();

  /**
   * @brief Read and publish the robot state while no control commands are received
   */
  void run_state_publisher();

  /**
   * @brief Run the joint velocities controller
   * that reads commands from the joint velocities subscription
   */
  void run_joint_velocities_controller();

  /**
   * @brief Run the joint torques controller
   * that reads commands from the joint torques subscription
   */
  void run_joint_torques_controller();

};

inline bool DoosanLightWeightInterface::is_connected() const {
  return this->connected_;
}

inline bool DoosanLightWeightInterface::is_shutdown() const {
  return this->shutdown_;
}

inline std::mutex& DoosanLightWeightInterface::get_mutex() {
  return this->mutex_;
}

inline void DoosanLightWeightInterface::print_state() const {
  std::cout << "Current robot joint state:" << std::endl;
  std::cout << "--------------------" << std::endl;
  std::cout << this->state_ << std::endl;
  std::cout << "--------------------" << std::endl;
  std::cout << "Commanded torque:" << std::endl;
  std::cout << "--------------------" << std::endl;
//  std::cout << this->command_.get_torques().transpose() << std::endl;
  std::cout << "####################" << std::endl;
}
}
