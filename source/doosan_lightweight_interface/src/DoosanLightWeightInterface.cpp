#include <unistd.h>
#include "doosan_lightweight_interface/DoosanLightWeightInterface.hpp"

using namespace communication_interfaces::sockets;
using namespace DRAFramework;
using namespace state_representation;

namespace doosanlwi {

DoosanLightWeightInterface::DoosanLightWeightInterface(
    std::string robot_ip, ZMQCombinedSocketsConfiguration sockets_configuration, std::string prefix
) :
    prefix_(std::move(prefix)),
    robot_ip_(std::move(robot_ip)),
    connected_(false),
    shutdown_(false),
    control_type_(JointStateVariable::ALL),
    sockets_(std::make_shared<ZMQPublisherSubscriber>(sockets_configuration)) {}

TOnMonitoringStateCB DoosanLightWeightInterface::get_cb() {
  return [this](const ROBOT_STATE eState) { return; };
}

void DoosanLightWeightInterface::init() {
  this->drfl_.set_on_monitoring_state(this->get_cb());
//  this->drfl_.open_connection(this->robot_ip_);
//  this->drfl_.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
//  this->drfl_.set_robot_system(ROBOT_SYSTEM_REAL);
//  this->drfl_.manage_access_control();

  // create connection to the robot
  this->connected_ = this->drfl_.connect_rt_control(this->robot_ip_);
  if (!this->connected_) {
    throw std::runtime_error("Could not connect to robot");
  }
  float period = 0.005;
  int losscount = 4;
  this->drfl_.set_rt_control_output("v1.0", period, losscount);
//  this->drfl_.set_on_rt_monitoring_data(rt_monitoring_data_cb);
  this->connected_ = this->drfl_.start_rt_control();
  if (!this->connected_) {
    throw std::runtime_error("Could not start realtime control");
  }

  if (this->prefix_.empty()) {
    this->prefix_ = "doosan_";
  }
  std::string robot_name = this->prefix_.substr(0, this->prefix_.length() - 1);
  std::vector<std::string> joint_names(NUMBER_OF_JOINT);
  for (std::size_t j = 0; j < joint_names.size(); ++j) {
    joint_names.at(j) = this->prefix_ + "joint" + std::to_string(j + 1);
  }
  this->state_ = JointState(robot_name, joint_names);
  this->command_ = JointState(robot_name, 7);

  this->last_command_ = std::chrono::steady_clock::now();

  float des[NUMBER_OF_JOINT] = {0.01, 0.01, 0.01, 0.01, 0.01, 1.0};
  float des_a[NUMBER_OF_JOINT] = {-10000.0, -10000.0, -10000.0, -10000.0, -10000.0, -10000.0};
  float data[NUMBER_OF_JOINT] = {0.0,};
  this->drfl_.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
  while (true) {
    // get current state
//    memcpy(data, this->drfl_.read_data_rt()->actual_joint_position, sizeof(float) * 6);
//    this->state_.set_positions(std::vector<double>{std::begin(data), std::end(data)});
//    memcpy(data, this->drfl_.read_data_rt()->actual_joint_velocity, sizeof(float) * 6);
//    this->state_.set_velocities(std::vector<double>{std::begin(data), std::end(data)});
//    memcpy(data, this->drfl_.read_data_rt()->actual_joint_torque, sizeof(float) * 6);
//    this->state_.set_torques(std::vector<double>{std::begin(data), std::end(data)});
//    this->print_state();
//    memcpy(data, this->drfl_.read_data_rt()->gravity_torque, sizeof(float) * 6);
//    this->drfl_.torque_rt(data, 4.0);
    this->drfl_.speedj_rt(des, des_a, 0.1);
    usleep(100);
//    // make trajectory
//    TrajectoryGenerator(q_d, q_dot_d); // Custom Trajectory Generation Function
//    // gravity compensation + pd control
//    for (int i = 0; i < 6; i++) {
//      trq_d[i] = trq_g[i] + kp[i] * (q_d[i] – q[i]) +kd[i] * (q_dot_d[i] – q_dot[i]);
//    }
//    drfl.torque_rt(trq_d, 0);
//    if (time > plan1.time) {
//      time = 0;
//      Drfl.stop(STOP_TYPE_SLOW);
//      break;
//    }
//    rt_task_wait_period(NULL); // RTOS function
  }
}

//void DoosanLightWeightInterface::reset_command() {
////  this->command_.control_type = std::vector<int>(7, static_cast<int>(network_interfaces::control_type_t::UNDEFINED));
//  this->command_.set_velocities(Eigen::VectorXd::Zero(6));
//  this->command_.set_accelerations(Eigen::VectorXd::Zero(6));
//  this->command_.set_torques(Eigen::VectorXd::Zero(6));
//}
//
//void DoosanLightWeightInterface::run_controller() {
//  if (this->is_connected()) {
//    // restart the controller unless the node is shutdown
//    while (!this->is_shutdown()) {
//      try {
//        switch (this->control_type_) {
//          case JointStateVariable::TORQUES:
//            std::cout << "Starting joint torque controller..." << std::endl;
//            this->run_joint_torques_controller();
//            break;
//          default:
//            std::cout << "Unimplemented control type!" << std::endl;
//            this->control_type_ = JointStateVariable::ALL;
//            [[fallthrough]];
//          case JointStateVariable::ALL:
//            std::cout << "Starting state publisher..." << std::endl;
//            this->run_state_publisher();
//            break;
//        }
//      } catch (const std::exception& ex) {
//        std::cerr << ex.what() << std::endl;
//      }
//      std::cerr << "Controller stopped but the node is still active, restarting..." << std::endl;
//      //flush and reset any remaining command messages
//      this->reset_command();
//      std::this_thread::sleep_for(std::chrono::seconds(1));
//    }
//  } else {
//    throw std::runtime_error("Robot not connected! Call the init function first.");
//  }
//}
//
//void FrankaLightWeightInterface::poll_external_command() {
//  if (network_interfaces::zmq::receive(this->command_, this->zmq_subscriber_)) {
//    if (this->command_.joint_state.is_empty()) {
//      throw std::runtime_error("Received joint command is empty.");
//    }
//    this->last_command_ = std::chrono::steady_clock::now();
//    const auto& control_type = this->command_.control_type.at(0);
//    for (auto type_iter = std::next(this->command_.control_type.begin());
//         type_iter != this->command_.control_type.end(); ++type_iter) {
//      if (*type_iter != control_type) {
//        throw std::runtime_error(
//            "Currently, only commands where all the joints have the same control type are supported.");
//      }
//    }
//    this->control_type_ = network_interfaces::control_type_t(control_type);
//  } else if (std::chrono::duration_cast<std::chrono::milliseconds>(
//      std::chrono::steady_clock::now() - this->last_command_).count() > this->command_timeout_.count()) {
//    this->reset_command();
//  }
//}

//void DoosanLightWeightInterface::publish_robot_state() {
//  return;
//}
//
//void DoosanLightWeightInterface::read_robot_state() {
//  LPROBOT_POSE positions = this->drfl_.get_current_posj();
//  for (int i = 0; i < NUM_JOINT; ++i) {
//    this->state_.set_position(positions->_fPosition[i], i);
//  }
//  LPROBOT_VEL velocities = this->drfl_.get_current_velj();
//  for (int i = 0; i < NUM_JOINT; ++i) {
//    this->state_.set_velocity(velocities->_fVelocity[i], i);
//  }
//  LPROBOT_FORCE torques = this->drfl_.get_joint_torque();
//  for (int i = 0; i < NUM_JOINT; ++i) {
//    this->state_.set_torque(torques->_fForce[i], i);
//  }
//}
//
//void DoosanLightWeightInterface::run_state_publisher() {
//  this->read_robot_state();
//
//}
//
//void FrankaLightWeightInterface::run_joint_velocities_controller() {
//  // Set additional parameters always before the control loop, NEVER in the control loop!
//
//  this->franka_robot_->setJointImpedance(this->joint_impedance_values_);
//
//  // Set collision behavior.
//  this->franka_robot_->setCollisionBehavior(
//      this->collision_behaviour_.ltta, this->collision_behaviour_.utta, this->collision_behaviour_.lttn,
//      this->collision_behaviour_.uttn, this->collision_behaviour_.lfta, this->collision_behaviour_.ufta,
//      this->collision_behaviour_.lftn, this->collision_behaviour_.uftn);
//
//  try {
//    this->franka_robot_->control(
//        [this](
//            const franka::RobotState& robot_state, franka::Duration
//        ) -> franka::JointVelocities {
//          // check the local socket for a velocity command
//          this->poll_external_command();
//
//          if (this->control_type_ != network_interfaces::control_type_t::VELOCITY) {
//            if (this->control_type_ == network_interfaces::control_type_t::UNDEFINED) {
//              throw franka::ControlException("Control type reset!");
//            }
//            throw IncompatibleControlTypeException("Control type changed!");
//          }
//
//          // lock mutex
//          std::lock_guard<std::mutex> lock(this->get_mutex());
//          // extract current state
//          this->read_robot_state(robot_state);
//
//          std::array<double, 7> velocities{};
//          Eigen::VectorXd::Map(&velocities[0], 7) = this->command_.joint_state.get_velocities().array();
//
//          // write the state out to the local socket
//          this->publish_robot_state();
//
//          //return velocities;
//          return velocities;
//        });
//  } catch (const franka::Exception& e) {
//    std::cerr << e.what() << std::endl;
//  }
//}
//
//void FrankaLightWeightInterface::run_joint_torques_controller() {
//  // Set additional parameters always before the control loop, NEVER in the control loop!
//
//  // Set collision behavior.
//  this->franka_robot_->setCollisionBehavior(
//      this->collision_behaviour_.ltta, this->collision_behaviour_.utta, this->collision_behaviour_.lttn,
//      this->collision_behaviour_.uttn, this->collision_behaviour_.lfta, this->collision_behaviour_.ufta,
//      this->collision_behaviour_.lftn, this->collision_behaviour_.uftn);
//
//  try {
//    this->franka_robot_->control(
//        [this](
//            const franka::RobotState& robot_state, franka::Duration
//        ) -> franka::Torques {
//          // check the local socket for a torque command
//          this->poll_external_command();
//
//          if (this->control_type_ != network_interfaces::control_type_t::EFFORT) {
//            if (this->control_type_ == network_interfaces::control_type_t::UNDEFINED) {
//              throw franka::ControlException("Control type reset!");
//            }
//            throw IncompatibleControlTypeException("Control type changed!");
//          }
//
//          // lock mutex
//          std::lock_guard<std::mutex> lock(this->get_mutex());
//          // extract current state
//          this->read_robot_state(robot_state);
//
//          // get the coriolis array
//          std::array<double, 7> coriolis_array = this->franka_model_->coriolis(robot_state);
//          Eigen::Map<const Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
//
//          // get the mass matrix
//          std::array<double, 49> mass_array = franka_model_->mass(robot_state);
//          Eigen::Map<const Eigen::Matrix<double, 7, 7> > mass(mass_array.data());
//
//          std::array<double, 7> torques{};
//          Eigen::VectorXd::Map(&torques[0], 7) = this->command_.joint_state.get_torques().array()
//              - this->joint_damping_gains_ * this->state_.joint_state.get_velocities().array() + coriolis.array();
//
//          // write the state out to the local socket
//          this->publish_robot_state();
//
//          //return torques;
//          return torques;
//        });
//  } catch (const franka::Exception& e) {
//    std::cerr << e.what() << std::endl;
//  }
//}
}// namespace doosanlwi
