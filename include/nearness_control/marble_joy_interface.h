#ifndef MARBLE_JOY_INTERFACE_H
#define MARBLE_JOY_INTERFACE_H

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
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>

namespace marble_joy_interface{

class marbleJoyInterface {
 public:
    marbleJoyInterface(const ros::NodeHandle &node_handle,
                            const ros::NodeHandle &private_node_handle);
    ~marbleJoyInterface() = default;

    void init();

    // FUNCTIONS //

    void joyconCb(const sensor_msgs::JoyConstPtr& joy_msg);

 private:
    // public ros node handle
    ros::NodeHandle nh_;
    // private ros node handle
    ros::NodeHandle pnh_;
    std::string node_name_{"node_name"};

    // SUBSCRIBERS //
    ros::Subscriber sub_bluetooth_joy_;

    // PUBLISHERS //
    ros::Publisher pub_estop_engage_;
    ros::Publisher pub_sf_clustering_debug_;
    ros::Publisher pub_radio_estop_disengage_;
    
    std_msgs::Bool radio_estop_reset_msg_;



}; // class SimpleNodeClass

}  // namespace demo

#endif  // DEMO_SIMPLE_CLASS_NODE_HPP
