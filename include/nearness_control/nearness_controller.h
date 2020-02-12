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
#include <nearness_control/ClusterMsg.h>
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

    void joyconCb(const sensor_msgs::JoyConstPtr& joy_msg);
    void odomCb(const nav_msgs::OdometryConstPtr& odom_msg);
    //void imuCb(const sensor_msgs::ImuConstPtr& imu_msg);
    void sonarHeightCb(const sensor_msgs::RangeConstPtr& range_msg);
    void nextWaypointCb(const geometry_msgs::PointStampedConstPtr& next_waypoint_msg);
    void terrainScanCb(const sensor_msgs::LaserScan::ConstPtr& terrain_scan_msg);
    void towerSafetyCb(const std_msgs::Int32ConstPtr& safety_msg);
    void beaconStopCb(const std_msgs::BoolConstPtr& beacon_stop_msg);
    void convertHLaserscan2CVMat(const sensor_msgs::LaserScanPtr h_laserscan_msg);
    void convertVLaserscan2CVMat(const sensor_msgs::LaserScanPtr v_laserscan_msg);
    void computeHorizFourierCoeffs();
    void computeVertFourierCoeffs();
    void computeForwardSpeedCommand();
    void computeWFYawRateCommand();
    void computeSFYawRateCommand();
    void computeTerrainYawRateCommand();
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
    ros::Subscriber sub_next_waypoint_;
    ros::Subscriber sub_terrain_scan_;
    ros::Subscriber sub_tower_safety_;
    ros::Subscriber sub_beacon_stop_;

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
    ros::Publisher pub_control_commands_stamped_;
    ros::Publisher pub_debug_weighting_;
    ros::Publisher pub_vehicle_status_;

    ros::Publisher pub_estop_engage_;
    ros::Publisher pub_sf_clustering_debug_;
    ros::Publisher pub_ter_clusters_;

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
    float sat(float num, float min_val, float max_val);

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
    bool flag_estop_;
    double safety_radius_;
    double f_dist_;
    double s_dist_;

    // Controller
    double u_k_hb_1_;
    double u_k_hb_2_;
    double u_k_ha_1_;
    double u_k_ha_2_;
    double u_k_att_;
    double u_k_vb_1_;
    double u_k_vb_2_;
    double u_max_;
    double u_min_;
    bool reverse_u_cmd_;
    double r_k_hb_1_;
    double r_k_hb_2_;
    double r_k_vb_1_;
    double r_k_vb_2_;
    double r_max_;
    bool reverse_r_cmd_;
    bool reverse_wf_r_cmd_;
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
    double r_k_att_turn_;
    bool enable_attractor_control_;
    bool is_ground_vehicle_;
    bool have_attractor_;
    bool enable_wf_control_;
    bool attractor_turn_;
    bool enable_command_weighting_;
    bool enable_att_speed_reg_;
    double attractor_latch_thresh_;
    bool enable_terrain_control_;
    bool enable_tower_safety_;

    // Init
    std::vector<float> h_gamma_vector_;
    std::vector<float> v_gamma_vector_;
    std::vector<float> safety_boundary_;
    int left_corner_index_;
    bool flag_too_close_front_;
    bool flag_too_close_side_;
    double close_side_speed_;
    float h_dg_;
    float v_dg_;
    float range_agl_;
    bool debug_;
    bool flag_beacon_stop_;

    // converHtLaserscan2CVMat
    int h_num_fourier_terms_;
    cv::Mat h_depth_cvmat_;

    // convertVLaserscan2CVMat
    int v_num_fourier_terms_;
    cv::Mat v_depth_cvmat_;

    // computeHorizFourierCoeffs
    float h_a_[10], h_b_[10];
    cv::Mat h_nearness_;
    float h_nearness_l2_norm_;

    // computeVertFourierCoeffs
    float v_a_[10], v_b_[10];
    cv::Mat v_nearness_;

    // computeSFYawRateCommand
    float h_sf_r_cmd_;
    float h_sf_nearness_l2_norm_;
    float h_nearness_maxval_;
    int num_sf_clusters_;
    bool enable_sf_clustering_;

    // computeAttractorCommand
    float attractor_yaw_cmd_;
    float attractor_d_;
    float relative_attractor_heading_;
    bool stagger_waypoints_;

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
    ros::Time last_wp_msg_time_;
    double attractor_watchdog_timer_;
    bool lost_attractor_;

    // odomCb
    geometry_msgs::Point current_pos_;
    double current_roll_;
    double current_pitch_;
    double current_heading_;

    // terrain
    int num_tscan_points_;
    std::vector<float> tscan_gamma_vector_;
    std::vector<float> ter_cluster_d_;
    std::vector<float> ter_cluster_r_;
    int num_ter_clusters_;
    double terrain_thresh_;
    bool flag_terrain_too_close_front_;
    double terrain_front_safety_radius_;
    float terrain_r_cmd_;

    // Twoer Safety
    bool flag_safety_too_close_;
    bool flag_safety_getting_close_;
    std::vector<int> safety_getting_close_counter_;
    std::vector<int> safety_too_close_counter_;
    int safety_getting_close_vote_thresh_;
    int safety_too_close_vote_thresh_;
    int safety_getting_close_num_votes_;
    int safety_too_close_num_votes_;
    int safety_counter1_;
    int safety_counter2_;


}; // class SimpleNodeClass

}  // namespace demo

#endif  // DEMO_SIMPLE_CLASS_NODE_HPP
