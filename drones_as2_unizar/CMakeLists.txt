cmake_minimum_required(VERSION 3.5)
project(drones_as2_unizar)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_python
  rclcpp
  rclpy
  rclcpp_components
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  mocap4r2_msgs
)
foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

add_library(gt_pose_broadcaster SHARED
  src/gt_pose_broadcaster_node.cpp)

target_include_directories(gt_pose_broadcaster
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  )

ament_target_dependencies(gt_pose_broadcaster
  PUBLIC
  rclcpp
  rclcpp_components
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  mocap4r2_msgs
  )

rclcpp_components_register_node(gt_pose_broadcaster PLUGIN "drones_as2_unizar::GTPoseBroadcaster" EXECUTABLE gt_pose_broadcaster_node)
install(TARGETS gt_pose_broadcaster
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_executable(vision_pose_republisher src/vision_pose_republisher.cpp)
ament_target_dependencies(vision_pose_republisher rclcpp geometry_msgs)
install(TARGETS
  vision_pose_republisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/keep_pose.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()