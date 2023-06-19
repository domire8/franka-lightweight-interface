#include <iostream>
#include <thread>
#include <chrono>

#undef NDEBUG
#include <cassert>

#include <DRFLEx.h>
#include <state_representation/space/joint/JointState.hpp>
#include <state_representation/space/joint/JointVelocities.hpp>
#include <clproto.hpp>
#include <communication_interfaces/sockets/ZMQPublisherSubscriber.hpp>

#define DEG_2_RAD (M_PI / 180.0)
#define RAD_2_DEG (180.0 / M_PI)

using namespace DRAFramework;

state_representation::JointState joint_state
    ("doosan", {"m0609_joint1", "m0609_joint2", "m0609_joint3", "m0609_joint4", "m0609_joint5", "m0609_joint6"});
auto context = std::make_shared<zmq::context_t>(1);
communication_interfaces::sockets::ZMQCombinedSocketsConfiguration config = {context, "*", "1901", "1902", true, true};
communication_interfaces::sockets::ZMQPublisherSubscriber sockets(config);

CDRFLEx drfl;
bool has_control_authority = false;
bool tp_init_completed = false;

void on_tp_initializing_completed() {
  tp_init_completed = drfl.manage_access_control(MANAGE_ACCESS_CONTROL_REQUEST);
  std::cout << "TP initialized" << std::endl;
}

void on_program_stopped(const PROGRAM_STOP_CAUSE) {
  assert(drfl.drl_stop(STOP_TYPE_SLOW));
}

void on_monitoring_state(const ROBOT_STATE eState) {
  switch (eState) {
    case STATE_INITIALIZING:
      std::cout << "STATE_INITIALIZING" << std::endl;
      if (has_control_authority) {
        drfl.set_robot_control(CONTROL_ENABLE_OPERATION);
      }
      break;
    case STATE_STANDBY:
      std::cout << "STATE_STANDBY" << std::endl;
      break;
    case STATE_MOVING:
      std::cout << "STATE_MOVING" << std::endl;
      break;
    case STATE_SAFE_OFF:
      std::cout << "STATE_SAFE_OFF" << std::endl;
      if (has_control_authority) {
        drfl.set_robot_control(CONTROL_SERVO_ON);
      }
      break;
    case STATE_TEACHING:
      std::cout << "STATE_STANDBY" << std::endl;
      break;
    case STATE_SAFE_STOP:
      std::cout << "STATE_SAFE_STOP" << std::endl;
      if (has_control_authority) {
        drfl.set_safe_stop_reset_type(SAFE_STOP_RESET_TYPE_DEFAULT);
        drfl.set_robot_control(CONTROL_RESET_SAFET_STOP);
      }
      break;
    case STATE_EMERGENCY_STOP:
      std::cout << "STATE_EMERGENCY_STOP:" << std::endl;
      break;
    case STATE_HOMMING:
      std::cout << "STATE_HOMMING:" << std::endl;
      break;
    case STATE_RECOVERY:
      std::cout << "STATE_RECOVERY:" << std::endl;
      // drfl.setrobot_control(CONTROL_RESET_RECOVERY);
      break;
    case STATE_SAFE_STOP2:
      std::cout << "STATE_SAFE_STOP2:" << std::endl;
      if (has_control_authority) {
        drfl.set_robot_control(CONTROL_RECOVERY_SAFE_STOP);
      }
      break;
    case STATE_SAFE_OFF2:
      std::cout << "STATE_SAFE_OFF2:" << std::endl;
      if (has_control_authority) {
        drfl.set_robot_control(CONTROL_RECOVERY_SAFE_OFF);
      }
      break;
    case STATE_NOT_READY:
      std::cout << "STATE_NOT_READY:" << std::endl;
      if (has_control_authority) {
        drfl.SetRobotControl(CONTROL_INIT_CONFIG);
      }
    default:
      break;
  }
}

void on_monitoring_access_control(const MONITORING_ACCESS_CONTROL eTransitControl) {
  switch (eTransitControl) {
    case MONITORING_ACCESS_CONTROL_REQUEST:
      std::cerr << "Control requested on pendant, granting..." << std::endl;
      assert(drfl.manage_access_control(MANAGE_ACCESS_CONTROL_RESPONSE_YES));
    case MONITORING_ACCESS_CONTROL_DENY:
      std::cerr << "Control denied, aborting..." << std::endl;
      exit(0);
    case MONITORING_ACCESS_CONTROL_LOSS:
      std::cerr << "Control lost, retrying..." << std::endl;
      has_control_authority = false;
      if (tp_init_completed) {
        drfl.manage_access_control(MANAGE_ACCESS_CONTROL_REQUEST);
      }
      break;
    case MONITORING_ACCESS_CONTROL_GRANT:
      has_control_authority = true;
      on_monitoring_state(drfl.get_robot_state());
      std::cout << "Control granted to lwi..." << std::endl;
      break;
    default:
      break;
  }
}

void on_log_alarm(LPLOG_ALARM tLog) {
  cout << "Alarm Info: " << "group(" << (unsigned int) tLog->_iGroup << "), index(" << tLog->_iIndex << "), param("
       << tLog->_szParam[0] << "), param(" << tLog->_szParam[1] << "), param(" << tLog->_szParam[2] << ")" << endl;
}

void on_rt_monitoring_data(LPRT_OUTPUT_DATA_LIST tData) {
  static int td = 0;
  for (int i = 0; i < NUMBER_OF_JOINT; ++i) {
    joint_state.set_position(DEG_2_RAD * tData->actual_joint_position[i], i);
    joint_state.set_velocity(DEG_2_RAD * tData->actual_joint_velocity[i], i);
    joint_state.set_torque(tData->gravity_torque[i], i);
  }
  auto message = clproto::encode(joint_state);
  sockets.send_bytes(message);

//  if (td++ == 1000) {
//    td = 0;
//    printf("timestamp : %.3f\n", tData->time_stamp);
//    std::cout << joint_state << std::endl;
//    printf(
//        "raw_joint_torque = %7.4f, %7.4f, %7.4f, %7.4f, %7.4f, %7.4f\n", tData->raw_joint_torque[0],
//        tData->raw_joint_torque[1], tData->raw_joint_torque[2], tData->raw_joint_torque[3], tData->raw_joint_torque[4],
//        tData->raw_joint_torque[5]);
//    printf(
//        "actual_joint_torque = %7.4f, %7.4f, %7.4f, %7.4f, %7.4f, %7.4f\n", tData->actual_joint_torque[0],
//        tData->actual_joint_torque[1], tData->actual_joint_torque[2], tData->actual_joint_torque[3],
//        tData->actual_joint_torque[4], tData->actual_joint_torque[5]);
//    printf(
//        "external_joint_torque = %7.4f, %7.4f, %7.4f, %7.4f, %7.4f, %7.4f\n", tData->external_joint_torque[0],
//        tData->external_joint_torque[1], tData->external_joint_torque[2], tData->external_joint_torque[3],
//        tData->external_joint_torque[4], tData->external_joint_torque[5]);
//  }
}

void on_disconnected() {
  while (!drfl.open_connection("192.168.137.100")) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

int main(int argc, char** argv) {
  sockets.open();

  drfl.set_on_monitoring_state(on_monitoring_state);
  drfl.set_on_monitoring_access_control(on_monitoring_access_control);
  drfl.set_on_tp_initializing_completed(on_tp_initializing_completed);
  drfl.set_on_log_alarm(on_log_alarm);
  drfl.set_on_rt_monitoring_data(on_rt_monitoring_data);

  drfl.set_on_program_stopped(on_program_stopped);
  drfl.set_on_disconnected(on_disconnected);

  assert(drfl.open_connection("192.168.137.100"));

  SYSTEM_VERSION tSysVersion = {
      '\0',
  };
  drfl.get_system_version(&tSysVersion);
  drfl.setup_monitoring_version(1);
  cout << "System version: " << tSysVersion._szController << endl;
  cout << "Library version: " << drfl.get_library_version() << endl;

  drfl.set_robot_control(CONTROL_SERVO_ON);
  while ((drfl.get_robot_state() != STATE_STANDBY) || !has_control_authority) {
    this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  assert(drfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS));
  assert(drfl.set_robot_system(ROBOT_SYSTEM_REAL));

  assert(drfl.connect_rt_control("192.168.137.100"));
  std::string rt_version = "v1.0";
  float period = 0.002;
  int loss_count = 4;
  drfl.set_rt_control_output(rt_version, period, loss_count);
  assert(drfl.start_rt_control());
  std::cout << "rt started" << std::endl;

  float tau_d[NUMBER_OF_JOINT] = {0.0,};
  float acc[NUMBER_OF_JOINT] = {-10000, -10000, -10000, -10000, -10000, -10000};
  drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
  while (true) {
    std::string msg;
    if (sockets.receive_bytes(msg)) {
      state_representation::JointVelocities command;
      if (clproto::decode(msg, command)) {
        for (int i = 0; i < NUMBER_OF_JOINT; ++i) {
          tau_d[i] = RAD_2_DEG * command.get_velocity(i);
        }
      drfl.speedj_rt(tau_d, acc, 0.5);
      }
    }
//    if (sockets.receive_bytes(msg)) {
//      state_representation::JointTorques command;
//      if (clproto::decode(msg, command)) {
//        std::cerr << command << std::endl;
//        std::cerr << command + joint_state << std::endl;
//        for (int i = 0; i < NUMBER_OF_JOINT; ++i) {
//          tau_d[i] = 5 * command.get_torque(i) + joint_state.get_torque(i);
////          tau_d[i] = command.get_torque(i);
//        }
//        drfl.torque_rt(tau_d, 0.002);
//      }
//    }
//    else if (joint_state) {
//      for (int i = 0; i < NUMBER_OF_JOINT; ++i) {
//        tau_d[i] = joint_state.get_torque(i);
//      }
//      drfl.torque_rt(tau_d, 0.0);
//      this_thread::sleep_for(std::chrono::microseconds(10));
//    }
  }

  drfl.close_connection();

  return 0;
}