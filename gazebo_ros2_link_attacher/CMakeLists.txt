cmake_minimum_required(VERSION 3.8)
project(gazebo_ros2_link_attacher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Attach.srv"
)

add_library(${PROJECT_NAME}_plugin SHARED
  src/gazebo_ros2_link_attacher.cpp
)

target_include_directories(${PROJECT_NAME}_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_plugin
  rclcpp
  gazebo_ros
)

rosidl_target_interfaces(${PROJECT_NAME}_plugin
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
)

install(TARGETS ${PROJECT_NAME}_plugin
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}/srv
)

install(PROGRAMS
  scripts/attach_sls.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
