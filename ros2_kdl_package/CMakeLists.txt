cmake_minimum_required(VERSION 3.8)
project(ros2_kdl_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(iiwa_bringup REQUIRED)
find_package(iiwa_description REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_ign_bridge REQUIRED)
find_package(aruco_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
##aggiunta 2c
find_package(ros_ign_interfaces REQUIRED)



## aggiunta 2c
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/UpdateMarkerPose.srv"
  "action/Ros2KdlTraj.action"
  DEPENDENCIES builtin_interfaces action_msgs
)

ament_export_dependencies(rosidl_default_runtime)


## aggiunta 2.c
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

## Parte aggiunta 1c - Action Client
add_library(action_client_lib SHARED
  src/action_client.cpp)

target_include_directories(action_client_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

ament_target_dependencies(action_client_lib
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  sensor_msgs
  unique_identifier_msgs
  action_msgs
)

# Linka il typesupport per action_client
target_link_libraries(action_client_lib "${cpp_typesupport_target}")

rclcpp_components_register_node(action_client_lib 
  PLUGIN "KdlActionClient"
  EXECUTABLE action_client)

## ROS2 KDL Node
add_executable(ros2_kdl_node src/ros2_kdl_node.cpp src/kdl_robot.cpp src/kdl_planner.cpp src/kdl_control.cpp)

## aggiunta 2.c
add_executable(marker_pose_service src/marker_pose_service.cpp)

##aggiunta 2.c
target_include_directories(marker_pose_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

##aggiunta 2.c
ament_target_dependencies(marker_pose_service
  rclcpp
  ros_ign_interfaces
  geometry_msgs
  # altri pacchetti se servono
)

target_include_directories(ros2_kdl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

target_compile_features(ros2_kdl_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(ros2_kdl_node 
  orocos_kdl
  rclcpp
  kdl_parser
  sensor_msgs
  std_msgs
  rclcpp_action
  rclcpp_components
  unique_identifier_msgs
  action_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  ##aggiunta 2.c
  ros_ign_interfaces
)

# CRITICO: Linka il typesupport anche per ros2_kdl_node
target_link_libraries(ros2_kdl_node "${cpp_typesupport_target}")

##aggiunta 2.c
target_link_libraries(marker_pose_service "${cpp_typesupport_target}")


rclcpp_components_register_node(ros2_kdl_node
  PLUGIN "Iiwa_pub_sub"
  EXECUTABLE ros2_kdl_node_exec)
  

# Installa i target

install(TARGETS 
  ros2_kdl_node
  action_client_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
  
## Example node
add_executable(ros2_kdl_node_example src/ros2_kdl_node_example.cpp)

target_include_directories(ros2_kdl_node_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})

target_compile_features(ros2_kdl_node_example PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(ros2_kdl_node_example 
  orocos_kdl
  rclcpp
)

##aggiunta 2.c
install(TARGETS marker_pose_service
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS ros2_kdl_node_example
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
