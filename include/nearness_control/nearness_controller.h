#ifndef NEARNESS_CONTROLLER_H
#define NEARNESS_CONTROLLER_H

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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>
//#include <nearness_control/FourierCoefsMsg.h>
#include <numeric>
#include <iterator>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <cv_bridge/cv_bridge.h>


#include <std_msgs/String.h>
#include <boost/thread.hpp>

#include <nearness_control/NearnessControllerConfig.h>
using namespace cv_bridge;
namespace nearness{

class NearnessController {
 public:
    NearnessController(const ros::NodeHandle &node_handle,
                            const ros::NodeHandle &private_node_handle);
    ~NearnessController() = default;

    void init();

    // FUNCTIONS //
    void horizLaserscanCb(const sensor_msgs::LaserScanPtr laserscan_msg);
    //void joyconCb(const sensor_msgs::JoyConstPtr& joy_msg);
    //void odomCb(const nav_msgs::OdometryConstPtr& odom_msg);
    //void imuCb(const sensor_msgs::ImuConstPtr& imu_msg);
    void sonarHeightCb(const sensor_msgs::RangeConstPtr& range_msg);
    void convertLaserscan2CVMat(const sensor_msgs::LaserScanPtr h_laserscan_msg);
    //void calc_h_wfi_fourier_coefficients(int h_width_cropped, float h_gamma_start_FOV, float h_gamma_end_FOV);
    //void calc_forward_velocity_command();
    //void calc_wf_yaw_rate_command();
    //void calc_sf_yaw_rate_command();
    //void pub_control_command_msg();

 private:
    // public ros node handle
    ros::NodeHandle nh_;
    // private ros node handle
    ros::NodeHandle pnh_;
    std::string node_name_{"node_name"};

    // SUBSCRIBERS //
    ros::Subscriber sub_horiz_laserscan_;
    ros::Subscriber sub_vert_laserscan_;
    ros::Subscriber sub_bluetooth_joy_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_sonar_height_;

    // PUBLISHERS //
    ros::Publisher pub_h_scan_reformat_;
    ros::Publisher pub_h_scan_nearness_;
    ros::Publisher pub_h_sf_nearness_;
    ros::Publisher pub_h_recon_wf_nearness_;
    ros::Publisher pub_h_fourier_coefficients_;

    ros::Publisher pub_control_commands_;
    ros::Publisher pub_sim_control_commands_;
    ros::Publisher pub_sf_control_commands_;
    ros::Publisher pub_vehicle_status_;

    // DYNAMIC RECONFIGURE //
    boost::mutex connect_mutex_;
    boost::recursive_mutex config_mutex_;
    typedef nearness_control::NearnessControllerConfig Config;
    typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
    boost::shared_ptr<ReconfigureServer> reconfigure_server_;
    Config config_;

    // FUNCTIONS //
    int sgn(double v);
    double shortest_angle_err(const float angle1, const float angle2);
    void generateSafetyBox();
    void checkSafetyBoundary(std::vector<float> scan);

    // GLOBAL VARIABLES //

    // Init
    int total_h_scan_points_;
    int num_h_scan_points_;
    float h_scan_limit_;
    std::vector<float> h_gamma_vector_;
    std::vector<float> safety_boundary_;
    bool enable_safety_boundary_;
    bool enable_safety_box_;
    float f_dist_;
    float s_dist_;
    int left_corner_index_;
    bool enable_safety_radius_;
    double safety_radius_;
    bool flag_too_close_;

    // Sensor parameters
    float sensor_max_dist_;
    float sensor_min_dist_;
    std_msgs::String scan_start_loc;
    int h_scan_start_index_;
    float sensor_min_noise_;
    cv::Mat h_depth_cvmat_;
    float range_agl_;
    bool reverse_h_scan_;

}; // class SimpleNodeClass

}  // namespace demo

#endif  // DEMO_SIMPLE_CLASS_NODE_HPP
