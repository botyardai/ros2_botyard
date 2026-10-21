cmake_minimum_required(VERSION 3.8)
project(dexhand_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-sign-compare")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# test
add_executable(test_pub_dexhand_controller test/test_pub_dexhand_controller.cpp)
ament_target_dependencies(test_pub_dexhand_controller rclcpp control_msgs)
add_executable(test_position test/test_position.cpp)
ament_target_dependencies(test_position
  rclcpp
  rclcpp_action
  control_msgs
)
install(
  TARGETS test_pub_dexhand_controller test_position
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# src
add_executable(parallel_cmd_to_joint_state src/parallel_cmd_to_joint_state.cpp)
ament_target_dependencies(parallel_cmd_to_joint_state rclcpp std_msgs sensor_msgs)
install(
  TARGETS parallel_cmd_to_joint_state
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
