cmake_minimum_required(VERSION 3.8)
project(sentry_decision_v2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  nav2_util
  nav2_lifecycle_manager
  nav2_msgs
  nav_msgs
  std_msgs
  geometry_msgs
  rclcpp_lifecycle 
  message_filters 
  robot_state_publisher 
  nav2_behavior_tree
  rosidl_default_generators
  auto_aim_interfaces
  tf2_ros
  tf2
  tf2_geometry_msgs
  Eigen3
  visualization_msgs
)

add_executable(decision_node src/decision_node.cpp
                              src/Json/jsoncpp.cpp)
ament_target_dependencies(
  decision_node
  ${dependencies}
)
target_include_directories(decision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(decision_node PUBLIC c_std_99 cxx_std_17) 

install(TARGETS decision_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch JsonFiles
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
