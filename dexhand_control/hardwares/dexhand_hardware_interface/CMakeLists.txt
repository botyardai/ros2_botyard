cmake_minimum_required(VERSION 3.16)
project(dexhand_hardware_interface)

set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-value")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-variable")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-cpp")

find_package(ament_cmake REQUIRED)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pal_statistics
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  dexhand_driver
  dexhand_interfaces
  yaml-cpp
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

if(TARGET dexhand_driver::dexhand_driver)
  set(DEXHAND_DRIVER_LINK_LIBRARIES dexhand_driver::dexhand_driver)
else()
  set(DEXHAND_DRIVER_LINK_LIBRARIES ${DEXHAND_DRIVER_LIBRARIES})
  include_directories(${DEXHAND_DRIVER_INCLUDE_DIRS})
  link_directories(${DEXHAND_DRIVER_LIBRARY_DIRS})
endif()

add_library(
  ${PROJECT_NAME}
  SHARED
  # if $ROS_DISTRO is humble, use the humble version of dexhand_hardware_interface
  $<$<STREQUAL:$ENV{ROS_DISTRO},humble>:src/dexhand_hardware_interface_humble.cpp>
  # otherwise use the default version
  $<$<NOT:$<STREQUAL:$ENV{ROS_DISTRO},humble>>:src/dexhand_hardware_interface.cpp>
)
target_include_directories( ${PROJECT_NAME}
  PUBLIC
  include
)
# 第三方自定义库/非ament_cmake/ros库
target_link_libraries(${PROJECT_NAME}
  ${DEXHAND_DRIVER_LINK_LIBRARIES}
  yaml-cpp
)
# 管理基于ros/ament_cmake导出的include路径、链接库
ament_target_dependencies( ${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(hardware_interface dexhand_hardware_interface.xml)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()
