#ifndef DATA_COLLECTOR_H
#define DATA_COLLECTOR_H

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

#include <Eigen/Core>

using namespace std;
using namespace Eigen;

namespace data_collector{

class dataCollector {
 public:
  dataCollector(const ros::NodeHandle &node_handle,
                          const ros::NodeHandle &private_node_handle);
  ~dataCollector() = default;

  void init();

private:
   // public ros node handle
   ros::NodeHandle nh_;
   // private ros node handle
   ros::NodeHandle pnh_;
   std::string node_name_{"node_name"};

   void cmdCb(const geometry_msgs::TwistConstPtr& cmd_msg);
   void odomCb(const nav_msgs::OdometryConstPtr& odom_msg);
   void imuCb(const sensor_msgs::ImuConstPtr& imu_msg);
   bool cmpf(float A, float B, float epsilon);

   ros::Subscriber sub_cmds_;
   ros::Subscriber sub_imu_;
   ros::Subscriber sub_odom_;
   ros::Publisher pub_synced_odom_;
   ros::Publisher pub_synced_cmds_;

   // Outputs
   nav_msgs::Odometry current_odom_;
   geometry_msgs::TwistStamped current_cmds_;

   geometry_msgs::Quaternion imu_orientation_;
   geometry_msgs::Twist ang_vels_;

   int publish_rate_;
   float publish_period_s_;
   ros::Time last_publish_time_;
   ros::Time sim_time_;
   float pub_dt_s_;
   bool first_pass_;
   bool have_new_odom_;

};
} // end namespace

#endif
