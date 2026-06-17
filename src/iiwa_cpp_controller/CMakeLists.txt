cmake_minimum_required(VERSION 3.8)
project(iiwa_cpp_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

set(WORKSPACE_FUNCTIONS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../include/functions)
set(WORKSPACE_FUNCTIONS_COMMON_DIR ${WORKSPACE_FUNCTIONS_DIR}/common)

if(TARGET JsonCpp::JsonCpp)
  set(JSONCPP_TARGET JsonCpp::JsonCpp)
elseif(TARGET jsoncpp_lib)
  set(JSONCPP_TARGET jsoncpp_lib)
elseif(TARGET jsoncpp_static)
  set(JSONCPP_TARGET jsoncpp_static)
else()
  message(FATAL_ERROR "No jsoncpp CMake target found")
endif()

add_library(iiwa_model_functions
  ${WORKSPACE_FUNCTIONS_DIR}/KukaKinematics.cpp
  ${WORKSPACE_FUNCTIONS_DIR}/GravityVector.cpp
  ${WORKSPACE_FUNCTIONS_DIR}/CoriolisMatrix.cpp
  ${WORKSPACE_FUNCTIONS_DIR}/FrictionTorque.cpp
)
target_include_directories(iiwa_model_functions PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${WORKSPACE_FUNCTIONS_DIR}
  ${WORKSPACE_FUNCTIONS_COMMON_DIR}
)
target_compile_features(iiwa_model_functions PUBLIC cxx_std_17)

add_executable(iiwa_impedance_controller_cpp src/iiwa_impedance_controller_cpp.cpp)
ament_target_dependencies(iiwa_impedance_controller_cpp
  rclcpp sensor_msgs std_msgs
)
target_link_libraries(iiwa_impedance_controller_cpp iiwa_model_functions)
target_compile_features(iiwa_impedance_controller_cpp PRIVATE cxx_std_17)

add_executable(gesture_ee_pose_node_cpp src/gesture_ee_pose_node_cpp.cpp)
ament_target_dependencies(gesture_ee_pose_node_cpp
  geometry_msgs rclcpp sensor_msgs std_msgs
)
target_link_libraries(gesture_ee_pose_node_cpp iiwa_model_functions ${JSONCPP_TARGET})
target_compile_features(gesture_ee_pose_node_cpp PRIVATE cxx_std_17)

install(TARGETS
  iiwa_impedance_controller_cpp
  gesture_ee_pose_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
