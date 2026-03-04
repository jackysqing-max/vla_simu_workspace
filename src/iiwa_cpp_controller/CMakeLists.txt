cmake_minimum_required(VERSION 3.8)
project(iiwa_cpp_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/functions 
  ${CMAKE_CURRENT_SOURCE_DIR}/include/functions/common
)

add_executable(iiwa_impedance_controller_cpp src/iiwa_impedance_controller_cpp.cpp)
ament_target_dependencies(iiwa_impedance_controller_cpp
  rclcpp sensor_msgs std_msgs
)

install(TARGETS iiwa_impedance_controller_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
