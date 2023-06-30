cmake_minimum_required(VERSION 3.9)
project(doosan_lightweight_interface VERSION 0.0.1)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  	set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  	set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(communication_interfaces REQUIRED)
find_package(control_libraries 7.0.0 REQUIRED COMPONENTS state_representation)
find_package(clproto 7.0.0 REQUIRED)
find_library(DOOSAN_LIB NAMES DRFL REQUIRED)
find_package(Poco REQUIRED Foundation Net)

include_directories(include)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} communication_interfaces clproto state_representation ${DOOSAN_LIB} Poco::Net Poco::Foundation)

install(TARGETS ${PROJECT_NAME}
		RUNTIME DESTINATION bin
)
