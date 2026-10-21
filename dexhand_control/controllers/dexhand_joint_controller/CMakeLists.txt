cmake_minimum_required(VERSION 3.8)
project(dexhand_joint_controller)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  forward_command_controller
  generate_parameter_library
  hardware_interface
  pal_statistics
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  dexhand_joint_controller_parameters
  src/dexhand_joint_controller_parameters.yaml
  # PARAMETER_FILE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/include/dexhand_joint_controller
)

add_library(dexhand_joint_controller SHARED
  src/dexhand_joint_controller.cpp
)
target_compile_features(dexhand_joint_controller PUBLIC cxx_std_17)
target_include_directories(dexhand_joint_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:include/dexhand_joint_controller>
)
ament_target_dependencies(dexhand_joint_controller
  controller_interface
  forward_command_controller
  generate_parameter_library
  hardware_interface
  pal_statistics
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)
target_link_libraries(dexhand_joint_controller
  dexhand_joint_controller_parameters
)
pluginlib_export_plugin_description_file(controller_interface dexhand_joint_controller_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/dexhand_joint_controller
)
install(
  TARGETS
    dexhand_joint_controller
    dexhand_joint_controller_parameters
  EXPORT export_dexhand_joint_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_dexhand_joint_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
