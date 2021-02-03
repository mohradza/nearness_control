#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

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
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <nearness_control_msgs/TrajList.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <lcd_pkg/PoseGraph.h>
#include <tf/tf.h>
#include <math.h>

using namespace std;
namespace trajectory_generator{

class trajectoryGenerator {
 public:
    trajectoryGenerator(const ros::NodeHandle &node_handle,
                            const ros::NodeHandle &private_node_handle);
    ~trajectoryGenerator() = default;

    void init();

    // FUNCTIONS //
    void publishTrajectory();
    void odomCb(const nav_msgs::OdometryConstPtr& odom_msg);
    float dist(const geometry_msgs::Point p1, const geometry_msgs::Point p2);

 private:
    // public ros node handle
    ros::NodeHandle nh_;
    // private ros node handle
    ros::NodeHandle pnh_;
    ros::NodeHandle priv_nh;
    std::string node_name_{"node_name"};

    // SUBSCRIBERS //
    ros::Subscriber sub_odom_;

    // PUBLISHERS //
    ros::Publisher pub_traj_;

    nav_msgs::Odometry odom_;
    geometry_msgs::Point odom_point_;

    vector<geometry_msgs::Point> traj_list_points_;
    nearness_control_msgs::TrajList traj_list_msg_;
    double traj_point_dist_thresh_;

    bool initialized_;
    long int count_;

}; // class SimpleNodeClass

}  // namespace demo

#endif  // DEMO_SIMPLE_CLASS_NODE_HPP
