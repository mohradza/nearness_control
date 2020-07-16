#ifndef TRAJECTORY_FOLLOWER_H
#define TRAJECTORY_FOLLOWER_H

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
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <math.h>

using namespace std;
namespace trajectory_follower{

class trajectoryFollower {
 public:
    trajectoryFollower(const ros::NodeHandle &node_handle,
                            const ros::NodeHandle &private_node_handle);
    ~trajectoryFollower() = default;

    void init();

    // FUNCTIONS //
    void trajCb(const visualization_msgs::MarkerArrayConstPtr& msg);
    void odomCb(const nav_msgs::OdometryConstPtr& odom_msg);
    void findNextLookahead();
    void publishLookahead();
    void taskCb(const std_msgs::String task_msg);
    float dist(const geometry_msgs::Point p1, const geometry_msgs::Point p2);
    bool doLookup();

 private:
    // public ros node handle
    ros::NodeHandle nh_;
    // private ros node handle
    ros::NodeHandle pnh_;
    ros::NodeHandle priv_nh;
    std::string node_name_{"node_name"};

    // SUBSCRIBERS //
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_traj_;
    ros::Subscriber sub_task_;

    // PUBLISHERS //
    ros::Publisher pub_lookahead_;

    nav_msgs::Odometry odom_;
    geometry_msgs::Point odom_point_;
    geometry_msgs::PointStamped lookahead_point_;

    vector<geometry_msgs::Point> traj_list_points_;


    ros::Time last_joy_msg_time_;

    int last_lookahead_index_;
    bool enable_lookahead_lookup_;
    bool have_current_traj_home_;

    int traj_list_size_;
    double lookahead_dist_;


}; // class SimpleNodeClass

}  // namespace demo

#endif  // DEMO_SIMPLE_CLASS_NODE_HPP
