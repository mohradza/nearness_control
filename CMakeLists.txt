cmake_minimum_required(VERSION 2.8.3)
project(nearness_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  tf2_ros
  pcl_conversions
  pcl_ros
  nearness_control_msgs
  visualization_msgs
  geometry_msgs
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   FourierCoefsMsg.msg
   ClusterMsg.msg
   )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   sensor_msgs
   geometry_msgs
   std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nearness_control
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime tf2_ros pcl_ros geometry_msgs visualization_msgs
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(projection_shape_generator src/projection_shape_generator.cpp)
target_link_libraries(projection_shape_generator ${catkin_LIBRARIES})

add_library(nearness_control_3d src/nearness_control_3d.cpp)
add_dependencies(nearness_control_3d ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(nearness_control_3d projection_shape_generator ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(nearness_control_3d_node src/nearness_control_3d_node.cpp)
target_link_libraries(nearness_control_3d_node nearness_control_3d ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(control_mixer_node src/control_mixer_node.cpp)
target_link_libraries(control_mixer_node ${catkin_LIBRARIES})

add_executable(control_mixer src/control_mixer.cpp)
target_link_libraries(control_mixer control_mixer_node ${catkin_LIBRARIES})

add_library(joy_teleop_mixer_node src/joy_teleop_mixer_node.cpp)
target_link_libraries(joy_teleop_mixer_node ${catkin_LIBRARIES})

add_executable(joy_teleop_mixer src/joy_teleop_mixer.cpp)
target_link_libraries(joy_teleop_mixer joy_teleop_mixer_node ${catkin_LIBRARIES})

add_executable(tf2odom src/tf2odom.cpp)
target_link_libraries(tf2odom ${catkin_LIBRARIES})

add_library(command_generator_node src/command_generator_node.cpp)
target_link_libraries(command_generator_node ${catkin_LIBRARIES})

add_executable(command_generator src/command_generator.cpp)
target_link_libraries(command_generator command_generator_node ${catkin_LIBRARIES})

add_library(data_collector_node src/data_collector_node.cpp)
target_link_libraries(data_collector_node ${catkin_LIBRARIES})

add_executable(data_collector src/data_collector.cpp)
target_link_libraries(data_collector data_collector_node ${catkin_LIBRARIES})


