#ifndef NEARNESS_CONTROL_3D_H
#define NEARNESS_CONTROL_3D_H

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
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nearness_control_msgs/ProjectionWithOdomMsg.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/extract_clusters.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>

#include <math.h>
#include <numeric>
#include <iterator>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <std_msgs/String.h>
#include <boost/thread.hpp>

#include <nearness_control/NearnessControl3DConfig.h>
using namespace cv_bridge;
using namespace std;
namespace nearness_3d{

class NearnessControl3D {
 public:
    NearnessControl3D(const ros::NodeHandle &node_handle,
                            const ros::NodeHandle &private_node_handle);
    ~NearnessControl3D() = default;

    void init();

    // FUNCTIONS //
    void pclCb(const sensor_msgs::PointCloud2ConstPtr& pcl_msg);
    void odomCb(const nav_msgs::OdometryConstPtr& odom_msg);
    void joyconCb(const sensor_msgs::JoyConstPtr& joy_msg);
    void enableControlCb(const std_msgs::Bool msg);
    void generateViewingAngleVectors();
    void generateProjectionShapes();
    void publishProjectionShapes();
    void processPcl();
    void projectNearness();
    void reconstructWideFieldNearness();
    void computeSmallFieldNearness();
    void computeControlCommands();
    void computeSFControlCommands();
    bool newPcl();
    bool isObstructedPoint(const float t, const float p);


 private:
    // public ros node handle
    ros::NodeHandle nh_;
    // private ros node handle
    ros::NodeHandle pnh_;
    std::string node_name_{"node_name"};

    // SUBSCRIBERS //
    ros::Subscriber sub_pcl_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_joy_;
    ros::Subscriber sub_enable_control_;

    ros::Publisher pub_pcl_;
    ros::Publisher pub_mu_pcl_;
    ros::Publisher pub_mu_pcl2_;
    ros::Publisher pub_dist_pcl_;
    ros::Publisher pub_Y00_;
    ros::Publisher pub_Y0p1_;
    ros::Publisher pub_Yp1p1_;
    ros::Publisher pub_Yn1p1_;
    ros::Publisher pub_Y0p2_;
    ros::Publisher pub_Yp1p2_;
    ros::Publisher pub_Yn1p2_;
    ros::Publisher pub_Yp2p2_;
    ros::Publisher pub_Yn2p2_;
    ros::Publisher pub_y_projection_shape_;
    ros::Publisher pub_theta_projection_shape_;
    ros::Publisher pub_z_projection_shape_;
    ros::Publisher pub_y_projections_with_odom_;
    ros::Publisher pub_recon_wf_mu_;
    ros::Publisher pub_sf_mu_;
    ros::Publisher pub_sf_thresh_;
    ros::Publisher pub_sf_filtered_;
    ros::Publisher pub_sf_cluster1_;
    ros::Publisher pub_sf_clusters_;
    ros::Publisher pub_control_commands_;
    ros::Publisher pub_cmd_markers_;

    // DYNAMIC RECONFIGURE //
    boost::mutex connect_mutex_;
    boost::recursive_mutex config_mutex_;
    typedef nearness_control::NearnessControl3DConfig Config;
    typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
    boost::shared_ptr<ReconfigureServer> reconfigure_server_;
    Config config_;
    void configCb(Config &config, uint32_t level);

    // FUNCTIONS //
    float sgn(double v);
    void saturateControls();
    float wrapAngle(float angle);
    float sat(float num, float min_val, float max_val);
    int fact(int n);

    bool enable_debug_;
    bool enable_altitude_hold_;
    bool enable_control_;
    bool enable_wf_control_;
    bool enable_sf_control_;
    bool enable_speed_regulation_;
    bool enable_cmd_scaling_;
    bool enable_analytic_shapes_;
    double test_ring_;
    bool new_pcl_;
    bool half_projections_;

    // GLOBAL VARIABLES //
    float phi_start_;
    float theta_start_;
    float dphi_;
    float dtheta_;
    string frame_id_;


    std::vector<float> phi_view_vec_;
    std::vector<float> theta_view_vec_;
    vector<vector<float>> viewing_angle_mat_;

    sensor_msgs::PointCloud2 pcl_out_msg_;
    sensor_msgs::PointCloud2 mu_out_msg_;

    pcl::PointCloud<pcl::PointXYZ> new_cloud_;
    pcl::PointCloud<pcl::PointXYZ> cloud_out_;
    pcl::PointCloud<pcl::PointXYZ> mu_cloud_out_;
    pcl::PointCloud<pcl::PointXYZ> recon_wf_cloud_out_;
    std::vector<float> mu_meas_;
    int pcl_width_;
    int pcl_height_;
    std_msgs::Header pcl_header_;
    int pcl_vertical_spread_;
    int num_ring_points_;
    int num_rings_;
    int num_excluded_rings_;
    int num_basis_shapes_;
    int last_index_;
    int num_wf_harmonics_;

    nav_msgs::Odometry current_odom_;

    vector<float> y_full_;
    vector<float> y_front_half_;
    vector<float> y_bottom_half_;
    std_msgs::Float32MultiArray y_projections_msg_;
    nearness_control_msgs::ProjectionWithOdomMsg y_projections_with_odom_msg_;

    vector<float> recon_wf_mu_vec_;
    pcl::PointCloud<pcl::PointXYZ> recon_wf_mu_pcl_;
    sensor_msgs::PointCloud2 recon_wf_mu_pcl_msg_;

    vector<float> sf_mu_;
    pcl::PointCloud<pcl::PointXYZ> sf_mu_pcl_;
    sensor_msgs::PointCloud2 sf_mu_pcl_msg_;

    pcl::PointCloud<pcl::PointXYZI> Y00_;
    pcl::PointCloud<pcl::PointXYZI> Y0p1_;
    pcl::PointCloud<pcl::PointXYZI> Yp1p1_;
    pcl::PointCloud<pcl::PointXYZI> Yn1p1_;
    pcl::PointCloud<pcl::PointXYZI> Y0p2_;
    pcl::PointCloud<pcl::PointXYZI> Yp1p2_;
    pcl::PointCloud<pcl::PointXYZI> Yn1p2_;
    pcl::PointCloud<pcl::PointXYZI> Yp2p2_;
    pcl::PointCloud<pcl::PointXYZI> Yn2p2_;

    sensor_msgs::PointCloud2 Y00_msg_;
    sensor_msgs::PointCloud2 Y0p1_msg_;
    sensor_msgs::PointCloud2 Yp1p1_msg_;
    sensor_msgs::PointCloud2 Yn1p1_msg_;
    sensor_msgs::PointCloud2 Y0p2_msg_;
    sensor_msgs::PointCloud2 Yp1p2_msg_;
    sensor_msgs::PointCloud2 Yn1p2_msg_;
    sensor_msgs::PointCloud2 Yn2p2_msg_;
    sensor_msgs::PointCloud2 Yp2p2_msg_;

    vector<vector<float>> shape_mat_;

    vector<float> Y00_vec_;
    vector<float> Y0p1_vec_;
    vector<float> Yp1p1_vec_;
    vector<float> Yn1p1_vec_;
    vector<float> Y0p2_vec_;
    vector<float> Yp1p2_vec_;
    vector<float> Yn1p2_vec_;
    vector<float> Yp2p2_vec_;
    vector<float> Yn2p2_vec_;

    vector<float> y_projection_shape_vec_;
    vector<float> theta_projection_shape_vec_;
    vector<float> z_projection_shape_vec_;
    pcl::PointCloud<pcl::PointXYZI> y_projection_shape_;
    pcl::PointCloud<pcl::PointXYZI> theta_projection_shape_;
    pcl::PointCloud<pcl::PointXYZI> z_projection_shape_;
    sensor_msgs::PointCloud2 y_projection_shape_msg_;
    sensor_msgs::PointCloud2 theta_projection_shape_msg_;
    sensor_msgs::PointCloud2 z_projection_shape_msg_;

    // Control
    vector<vector<float>> C_mat_;
    vector<float> C_y_;
    vector<float> C_z_;
    vector<float> C_theta_;
    vector<float> u_vec_;
    double r_;
    double u_u_, u_v_, u_r_, u_w_;
    double k_v_, k_r_, k_w_, k_alt_;
    double k_u_v_, k_u_r_;
    geometry_msgs::Twist control_commands_;
    geometry_msgs::Point current_pos_;
    double current_roll_, current_pitch_, current_heading_;
    double current_height_agl_;
    double reference_altitude_;
    double max_forward_speed_, max_lateral_speed_;
    double max_vertical_speed_, max_yaw_rate_;
    double forward_speed_;
    double front_mu_ave_;
    double max_lateral_nearness_;

    visualization_msgs::Marker u_cmd_marker_;
    visualization_msgs::Marker v_cmd_marker_;
    visualization_msgs::Marker w_cmd_marker_;
    visualization_msgs::Marker r_cmd_marker_;
    visualization_msgs::MarkerArray cmd_markers_;

    // Small field commands
    float sf_w_cmd_;
    float sf_v_cmd_;
    double sf_k_v_, sf_k_w_, sf_k_angle_, sf_k_mu_;

    // Joystick
    geometry_msgs::Twist joy_cmd_;
    bool sim_control_;
    bool use_observed_shapes_;


}; // class SimpleNodeClass

}  // namespace demo

#endif  // DEMO_SIMPLE_CLASS_NODE_HPP
