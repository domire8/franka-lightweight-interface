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

#include <ctime> /* clock_gettime() */
#include <sys/mman.h> /* mlockall() */
#include <sched.h> /* sched_setscheduler() */
/** Task period in ns. */
#define PERIOD_NS   (2000000) // period is 2ms

#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is
                                     guranteed safe to access without
                                     faulting */
#define NSEC_PER_SEC (1000000000)
#define FREQUENCY (NSEC_PER_SEC / PERIOD_NS)

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
        drfl.set_robot_control(CONTROL_INIT_CONFIG);
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

void stack_prefault(void) {
  unsigned char dummy[MAX_SAFE_STACK];

  memset(dummy, 0, MAX_SAFE_STACK);
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

//  float Cog[3] = {0, -38.63, 98.980};
//  float inertia[6] = {0, 0, 0, 0, 0, 0};
//  assert(drfl.add_tool("tool", 1.850f, Cog, inertia));
//  assert(drfl.set_tool("tool"));
//
//  float tcp[6] = {0, 0, 0.185, 0, 0, 0};
//  assert(drfl.add_tcp("tcp", tcp));
//  assert(drfl.set_tcp("tcp"));

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

  state_representation::JointState command, buffered_command;

  float tau_d[NUMBER_OF_JOINT] = {0.0,};
  float acc[NUMBER_OF_JOINT] = {-10000, -10000, -10000, -10000, -10000, -10000};
  drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);

  /* Set priority */
  struct sched_param param = {};
  param.sched_priority = sched_get_priority_max(SCHED_FIFO);

  printf("Using priority %i.\n", param.sched_priority);
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    perror("sched_setscheduler failed");
  }

  /* Lock memory */

  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    fprintf(stderr, "Warning: Failed to lock memory: %s\n", strerror(errno));
  }

  stack_prefault();

  printf("Starting RT task with dt=%u ns.\n", PERIOD_NS);

  struct timespec wakeup_time;
  clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
  wakeup_time.tv_sec += 1; /* start in future */
  wakeup_time.tv_nsec = 0;

  int ret = 0;
  bool end_cyclic_task = false;
  bool ignore_interrupt = false;
  while (!end_cyclic_task) {
    ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, nullptr);
    if (ret && !(ret == EINTR && ignore_interrupt)) {
      fprintf(stderr, "clock_nanosleep(): %s\n", strerror(ret));
      return ret;
    }

    std::string msg;
    clproto::MessageType command_type;
    if (sockets.receive_bytes(msg)) {
      command_type = clproto::check_message_type(msg);
      switch (command_type) {
        case clproto::MessageType::JOINT_VELOCITIES_MESSAGE:
          command = clproto::decode<state_representation::JointVelocities>(msg);
          buffered_command = command;
          break;
        case clproto::MessageType::JOINT_TORQUES_MESSAGE:
          command = clproto::decode<state_representation::JointTorques>(msg);
          buffered_command = command;
          break;
        default:
          std::cout << "unsupported message type" << std::endl;
          return 1;
      }
    }
    if (command && command.get_age() > 0.01) {
      buffered_command *= 0.1;
    }
    if (buffered_command) {
      switch (command_type) {
        case clproto::MessageType::JOINT_VELOCITIES_MESSAGE:
          for (int i = 0; i < NUMBER_OF_JOINT; ++i) {
            tau_d[i] = RAD_2_DEG * buffered_command.get_velocity(i);
          }
          drfl.speedj_rt(tau_d, acc, 0.01);
          break;
        case clproto::MessageType::JOINT_TORQUES_MESSAGE:
          for (int i = 0; i < NUMBER_OF_JOINT; ++i) {
            tau_d[i] = joint_state.get_torque(i) + buffered_command.get_torque(i);
          }
          drfl.torque_rt(tau_d, 0.01);
          break;
        default:
          std::cout << "unsupported message type" << std::endl;
          return 1;
      }
    }

    wakeup_time.tv_nsec += PERIOD_NS;
    while (wakeup_time.tv_nsec >= NSEC_PER_SEC) {
      wakeup_time.tv_nsec -= NSEC_PER_SEC;
      wakeup_time.tv_sec++;
    }
  }

  drfl.close_connection();

  return 0;
}