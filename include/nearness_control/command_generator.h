#ifndef COMMAND_GENERATOR_H
#define COMMAND_GENERATOR_H

#include <dynamic_reconfigure/server.h>
#include <iostream>
#include <random>
#include <ros/ros.h>

#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif
#include <boost/circular_buffer.hpp>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Core>

using namespace std;
using namespace Eigen;

namespace command_generator {

class commandGenerator {
public:
  commandGenerator(const ros::NodeHandle &node_handle,
                   const ros::NodeHandle &private_node_handle);
  ~commandGenerator() = default;

  void init();

  void publishControlCommands();

private:
  // public ros node handle
  ros::NodeHandle nh_;
  // private ros node handle
  ros::NodeHandle pnh_;
  std::string node_name_{"node_name"};

  void stateCb(const std_msgs::String state_msg);
  void odomCb(const nav_msgs::OdometryConstPtr &odom_msg);
  void generateCommandVel();
  void generateDoubletsCommand();
  void generateSwerveCommands();
  void generateDoubleConstCommands();

  ros::Subscriber sub_state_;
  ros::Subscriber sub_odom_;
  ros::Publisher pub_cmd_vel_;
  ros::Publisher pub_cmd_vel_stamped_;

  string state_;
  string routine_;

  bool start_doublets_;
  ros::Time start_doublets_time_;

  bool start_swerve_;
  ros::Time start_swerve_time_;
  bool swerve_complete_;
  double swerve_dur_;
  bool start_fwd_motion_;
  ros::Time fwd_motion_time_;

  double doublet_period_;
  double doublet_amplitude_;
  bool first_doublet_complete_;

  geometry_msgs::Twist cmd_vel_msg_;

  geometry_msgs::Point starting_point_;
  geometry_msgs::Point goal_point_;
  float goal_heading_;
  float starting_heading_;
  nav_msgs::Odometry current_odom_;
  geometry_msgs::Point current_pos_;
  double current_roll_, current_pitch_, current_heading_;

  double k_u_, k_v_, k_r_, k_w_;
  double xv_kp1_, xv_k_, u_y_, u_v_, u_r_, u_z_, u_w_;
  double p_, q_, r_;
  double c1_, c2_;

  Matrix<float, 10, 10> Mv_A_;
  Matrix<float, 10, 2> Mv_B_;
  Matrix<float, 1, 10> Mv_C_;
  Matrix<float, 10, 1> Mv_Xkp1_;
  Matrix<float, 10, 1> Mv_Xk_;

  Matrix<float, 4, 4> Mr_A_;
  Matrix<float, 4, 1> Mr_B_;
  Matrix<float, 1, 4> Mr_C_;
  Matrix<float, 4, 1> Mr_Xkp1_;
  Matrix<float, 4, 1> Mr_Xk_;

  Matrix<float, 4, 4> Mw_A_;
  Matrix<float, 4, 1> Mw_B_;
  Matrix<float, 1, 4> Mw_C_;
  Matrix<float, 4, 1> Mw_Xkp1_;
  Matrix<float, 4, 1> Mw_Xk_;

  double forward_speed_;
};
} // namespace command_generator

#endif
