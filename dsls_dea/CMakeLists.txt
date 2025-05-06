cmake_minimum_required(VERSION 3.8)
project(dsls_dea)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(dsls_dea_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# add executables
add_executable(dsls_dea_exe
  src/main_node3.cpp 

  # for inner loop controller when using main_node2.cpp
  # src/nonlinear_attitude_control.cpp

  # MATLAB generated files
  src/DSLSDEAController/DSLSDEAController.cpp
  src/DSLSDEAController/rt_nonfinite.cpp
  src/DSLSDEAController/rtGetInf.cpp
  src/DSLSDEAController/rtGetNaN.cpp
)

ament_target_dependencies(dsls_dea_exe 
  rclcpp 
  px4_msgs 
  gazebo_msgs 
  geometry_msgs 
  px4_ros_com
  dsls_dea_msgs
  tf2
  tf2_geometry_msgs
)

# Make sure Eigen's include path is used
target_include_directories(dsls_dea_exe PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${px4_ros_com_INCLUDE_DIRS}  # Add PX4 frame transforms
)

# find executables
install(TARGETS dsls_dea_exe DESTINATION lib/${PROJECT_NAME})

############
# Install ##
############
# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp px4_msgs)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

# Install launch files
install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

# install models
install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
