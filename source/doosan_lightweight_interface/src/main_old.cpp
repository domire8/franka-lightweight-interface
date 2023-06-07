#include <iostream>

#include "doosan_lightweight_interface/DoosanLightWeightInterface.hpp"

using namespace doosanlwi;

// Return the value of a named option from the argument buffer
// (for example, `--key value` will point to "value" for supplied option "key")
char* parse_option(char** begin, char** end, const std::string& option) {
  char** itr = std::find(begin, end, option);
  if (itr != end && ++itr != end) {
    return *itr;
  }
  return nullptr;
}

int main(int argc, char** argv) {
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

//  std::string help_message = "Usage: franka_lightweight_interface robot-id prefix ";
//  help_message += "[--joint-damping <high|medium|low|off>] [--sensitivity <high|medium|low>] "
//                  "[--joint-impedance <high|medium|low>]";
//  std::string robot_ip = "172.16.0.2";
//  std::string state_uri = "0.0.0.0:1601";
//  std::string command_uri = "0.0.0.0:1602";
//
//  if (argc <= 2) {
//    std::cerr << "Not enough input arguments. Provide at least the robot number and its prefix." << std::endl
//              << help_message << std::endl;
//    return 1;
//  }
//  if (atof(argv[1]) == 17) {
//    robot_ip = "172.17.0.2";
//    state_uri = "0.0.0.0:1701";
//    command_uri = "0.0.0.0:1702";
//  } else if (atof(argv[1]) != 16) {
//    std::cerr << "This robot is unknown, choose either '16' or '17'." << std::endl << help_message << std::endl;
//    return 1;
//  }
//  std::string prefix = argv[2];
//  if (prefix.substr(prefix.length() - 1, 1) != "_") {
//    std::cerr << "Please provide a prefix that ends with an underscore." << std::endl << help_message << std::endl;
//    return 1;
//  }

  auto context = std::make_shared<zmq::context_t>(1);
  communication_interfaces::sockets::ZMQCombinedSocketsConfiguration sockets_configuration = {
      context, "0.0.0.0", "1701", "1702"
  };
  DoosanLightWeightInterface dlwi("192.168.137.100", sockets_configuration, "doosan_");

  dlwi.init();
//  dlwi.run_controller();
  return 0;
}