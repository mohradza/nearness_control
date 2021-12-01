#ifndef COMMAND_GENERATOR_H
#define COMMAND_GENERATOR_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <iostream>
#include <random>

#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif
#include <boost/circular_buffer.hpp>


#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

using namespace std;

namespace command_generator{

class commandGenerator {
 public:
  commandGenerator(const ros::NodeHandle &node_handle,
                          const ros::NodeHandle &private_node_handle);
  ~commandGenerator() = default;

  void init();

  void stateCb(const std_msgs::String state_msg);
  void publishControlCommands();
  void generateCommandVel();
  void generateDoubletsCommand();

  string state_;
  string routine_;

  bool start_doublets_;
  ros::Time start_doublets_time_;

  double doublet_period_;
  double doublet_amplitude_;
  bool first_doublet_complete_;

  geometry_msgs::Twist cmd_vel_msg_;

  geometry_msgs::Point starting_point_;
}
