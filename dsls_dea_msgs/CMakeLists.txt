cmake_minimum_required(VERSION 3.8)
project(dsls_dea_msgs)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# for customize msg and srv
find_package(rosidl_default_generators REQUIRED)

# for generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DSlsState.msg"
  "msg/DEAState.msg"
  "msg/LPFData.msg"
  DEPENDENCIES geometry_msgs std_msgs 
)

# ament_export_dependencies(rosidl_default_runtime)
ament_package()
