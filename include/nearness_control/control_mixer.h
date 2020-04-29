#ifndef CONTROL_MIXER_H
#define CONTROL_MIXER_H

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
#include <tf/tf.h>

namespace control_mixer{

class controlMixer {
 public:
    controlMixer(const ros::NodeHandle &node_handle,
                            const ros::NodeHandle &private_node_handle);
    ~controlMixer() = default;

    void init();

    // FUNCTIONS //

    void joyconCb(const geometry_msgs::TwistConstPtr& joy_msg);
    void odomCb (const nav_msgs::OdometryConstPtr& odom_msg);

 private:
    // public ros node handle
    ros::NodeHandle nh_;
    // private ros node handle
    ros::NodeHandle pnh_;
    ros::NodeHandle priv_nh;
    std::string node_name_{"node_name"};

    // SUBSCRIBERS //
    ros::Subscriber sub_joy_;
    ros::Subscriber sub_odom_;

    // PUBLISHERS //
    ros::Publisher pub_cmd_;

    geometry_msgs::Twist joy_cmd_;
    nav_msgs::Odometry odom_;
    float ref_h_;
    float ref_heading_;
    float k_height_;
    float k_heading_;
    bool enable_joy_thrust_control_;
    bool enable_heading_controller_;


    ros::Time last_joy_msg_time_;



}; // class SimpleNodeClass

}  // namespace demo

#endif  // DEMO_SIMPLE_CLASS_NODE_HPP
