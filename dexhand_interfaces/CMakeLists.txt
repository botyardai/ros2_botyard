cmake_minimum_required(VERSION 3.8)
project(dexhand_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TactilePoint.msg"
  "msg/FingerTactileGrid.msg"
  "msg/TactileGrid.msg"
  "action/DoGesture.action"
  DEPENDENCIES
  std_msgs
  action_msgs
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gesture_enums test/test_gesture_enums.cpp)
  target_include_directories(test_gesture_enums PRIVATE include)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs)
ament_package()
