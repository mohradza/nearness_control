cmake_minimum_required(VERSION 2.8.3)
project(nearness_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  cv_bridge
  tf2_ros
  pcl_conversions
  pcl_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
#find_package(Boost REQUIRED COMPONENTS system filesystem)

if(cv_bridge_VERSION VERSION_GREATER "1.12.0")
  add_compile_options(-std=c++11)
endif()

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

generate_dynamic_reconfigure_options(
	cfg/NearnessController.cfg
  cfg/NearnessController3D.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nearness_control
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs cv_bridge message_runtime tf2_ros pcl_ros
  DEPENDS OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(nearness_controller src/nearness_controller.cpp)
add_dependencies(nearness_controller ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(nearness_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(nearness_controller_node src/nearness_controller_node.cpp)
target_link_libraries(nearness_controller_node nearness_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_library(nearness_control_3d src/nearness_control_3d.cpp)
add_dependencies(nearness_control_3d ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(nearness_control_3d ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(nearness_control_3d_node src/nearness_control_3d_node.cpp)
target_link_libraries(nearness_control_3d_node nearness_control_3d ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(control_mixer_node src/control_mixer_node.cpp)
target_link_libraries(control_mixer_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(control_mixer src/control_mixer.cpp)
#add_dependencies(control_mixer ${PROJECT_NAME}_gencfg )
target_link_libraries(control_mixer control_mixer_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(joy_teleop_mixer_node src/joy_teleop_mixer_node.cpp)
target_link_libraries(joy_teleop_mixer_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(joy_teleop_mixer src/joy_teleop_mixer.cpp)
#add_dependencies(joy_teleop_mixer ${PROJECT_NAME}_gencfg )
target_link_libraries(joy_teleop_mixer joy_teleop_mixer_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
