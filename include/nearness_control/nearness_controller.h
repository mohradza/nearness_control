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
#include <nearness_control/FourierCoefsMsg.h>
#include <math.h>
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
    void horizLaserscanCb(const sensor_msgs::LaserScanPtr h_laserscan_msg);
    void vertLaserscanCb(const sensor_msgs::LaserScanPtr v_laserscan_msg);

    //void joyconCb(const sensor_msgs::JoyConstPtr& joy_msg);
    void odomCb(const nav_msgs::OdometryConstPtr& odom_msg);
    //void imuCb(const sensor_msgs::ImuConstPtr& imu_msg);
    void sonarHeightCb(const sensor_msgs::RangeConstPtr& range_msg);
    void nextWaypointCb(const geometry_msgs::PointStampedConstPtr& next_waypoint_msg);
    void convertHLaserscan2CVMat(const sensor_msgs::LaserScanPtr h_laserscan_msg);
    void convertVLaserscan2CVMat(const sensor_msgs::LaserScanPtr v_laserscan_msg);
    void computeHorizFourierCoeffs();
    void computeVertFourierCoeffs();
    void computeForwardSpeedCommand();
    void computeWFYawRateCommand();
    void computeSFYawRateCommand();
    void computeAttractorCommand();
    void computeLateralSpeedCommand();
    void computeWFVerticalSpeedCommand();
    void computeSFVerticalSpeedCommand();
    void publishControlCommandMsg();
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
    ros::Subscriber sub_next_waypoint_;

    // PUBLISHERS //
    ros::Publisher pub_h_scan_reformat_;
    ros::Publisher pub_h_scan_nearness_;
    ros::Publisher pub_h_sf_nearness_;
    ros::Publisher pub_h_recon_wf_nearness_;
    ros::Publisher pub_h_fourier_coefficients_;
    ros::Publisher pub_h_sf_yawrate_command_;

    ros::Publisher pub_v_scan_reformat_;
    ros::Publisher pub_v_scan_nearness_;
    ros::Publisher pub_v_sf_nearness_;
    ros::Publisher pub_v_recon_wf_nearness_;
    ros::Publisher pub_v_fourier_coefficients_;
    ros::Publisher pub_v_sf_vertspeed_command_;


    ros::Publisher pub_control_commands_;
    ros::Publisher pub_sim_control_commands_;
    ros::Publisher pub_vehicle_status_;

    // DYNAMIC RECONFIGURE //
    boost::mutex connect_mutex_;
    boost::recursive_mutex config_mutex_;
    typedef nearness_control::NearnessControllerConfig Config;
    typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
    boost::shared_ptr<ReconfigureServer> reconfigure_server_;
    Config config_;
    void configCb(Config &config, uint32_t level);


    // FUNCTIONS //
    float sgn(double v);
    double shortest_angle_err(const float angle1, const float angle2);
    void generateSafetyBox();
    void checkSafetyBoundary(std::vector<float> scan);
    void saturateControls();
    float wrapAngle(float angle);

    // GLOBAL VARIABLES //

    // PARAMETERS
    // Sensor - Horizontal
    int total_h_scan_points_;
    int num_h_scan_points_;
    double h_scan_limit_;
    std_msgs::String h_scan_start_loc_;
    double h_sensor_max_dist_;
    double h_sensor_min_dist_;
    int h_scan_start_index_;
    double h_sensor_min_noise_;
    bool reverse_h_scan_;
    // Sensor - Vertical
    int total_v_scan_points_;
    int num_v_scan_points_;
    double v_scan_limit_;
    std_msgs::String v_scan_start_loc_;
    double v_sensor_max_dist_;
    double v_sensor_min_dist_;
    int v_scan_start_index_;
    double v_sensor_min_noise_;
    bool reverse_v_scan_;

    // Safety
    bool enable_safety_boundary_;
    bool enable_safety_box_;
    bool enable_safety_radius_;
    bool enable_sf_control_;
    double safety_radius_;
    double f_dist_;
    double s_dist_;

    // Controller
    double u_k_hb_1_;
    double u_k_hb_2_;
    double u_k_vb_1_;
    double u_k_vb_2_;
    double u_max_;
    double u_min_;
    double r_k_hb_1_;
    double r_k_hb_2_;
    double r_k_vb_1_;
    double r_k_vb_2_;
    double r_max_;
    double h_sf_k_0_;
    double h_sf_k_d_;
    double h_sf_k_psi_;
    double h_sf_k_thresh_;
    double v_sf_k_0_;
    double v_sf_k_d_;
    double v_sf_k_psi_;
    double v_sf_k_thresh_;
    double v_sf_w_cmd_;
    double v_k_hb_1_;
    double v_max_;
    double w_k_1_;
    double w_k_2_;
    double w_max_;
    double r_k_att_0_;
    double r_k_att_d_;
    bool enable_gain_scaling_;
    bool enable_attractor_control_;
    bool is_ground_vehicle_;
    bool have_attractor_;
    bool enable_wf_control_;


    // Init
    std::vector<float> h_gamma_vector_;
    std::vector<float> v_gamma_vector_;
    std::vector<float> safety_boundary_;
    int left_corner_index_;
    bool flag_too_close_;
    float h_dg_;
    float v_dg_;
    float range_agl_;
    bool debug_;

    // converHtLaserscan2CVMat
    int h_num_fourier_terms_;
    cv::Mat h_depth_cvmat_;

    // convertVLaserscan2CVMat
    int v_num_fourier_terms_;
    cv::Mat v_depth_cvmat_;

    // computeHorizFourierCoeffs
    float h_a_[10], h_b_[10];
    cv::Mat h_nearness_;

    // computeVertFourierCoeffs
    float v_a_[10], v_b_[10];
    cv::Mat v_nearness_;

    // computeSFYawRateCommand
    float h_sf_r_cmd_;

    // computeAttractorCommand
    float attractor_yaw_cmd_;

    // computeForwardSpeedCommand
    float u_cmd_;

    // computeWFYawRateCommand
    float h_wf_r_cmd_;

    // computeLateralSpeedCommand
    float h_wf_v_cmd_;

    // computeVertFourierCoeffs
    float v_wf_w_cmd_;

    // publishControlCommandMsg
    geometry_msgs::TwistStamped control_command_;

    // nextWaypointCb
    geometry_msgs::Point next_waypoint_pos_;

    // odomCb
    geometry_msgs::Point current_pos_;
    double current_roll_;
    double current_pitch_;
    double current_heading_;


}; // class SimpleNodeClass

}  // namespace demo

#endif  // DEMO_SIMPLE_CLASS_NODE_HPP
