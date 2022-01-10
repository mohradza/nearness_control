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
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>

using namespace std;

namespace command_generator{

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
   void odomCb(const nav_msgs::OdometryConstPtr& odom_msg);
   void generateCommandVel();
   void generateDoubletsCommand();

   ros::Subscriber sub_state_;
   ros::Subscriber sub_odom_;
   ros::Publisher pub_cmd_vel_;

   string state_;
   string routine_;

   bool start_doublets_;
   ros::Time start_doublets_time_;

   double doublet_period_;
   double doublet_amplitude_;
   bool first_doublet_complete_;

   geometry_msgs::Twist cmd_vel_msg_;

   geometry_msgs::Point starting_point_;
   float starting_heading_;
   nav_msgs::Odometry current_odom_;
   geometry_msgs::Point current_pos_;
   double current_roll_, current_pitch_, current_heading_;

   double k_u_, k_v_, k_r_, k_w_;
   double xv_kp1_, xv_k_, u_y_, u_v_;
};
} // end namespace

#endif
