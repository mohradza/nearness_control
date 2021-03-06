#ifndef NEARNESS_CONTROLLER_3D_H
#define NEARNESS_CONTROLLER_3D_H

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
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

#include <nearness_control/NearnessController3DConfig.h>
using namespace cv_bridge;
using namespace std;
namespace nearness_3d{

class NearnessController3D {
 public:
    NearnessController3D(const ros::NodeHandle &node_handle,
                            const ros::NodeHandle &private_node_handle);
    ~NearnessController3D() = default;

    void init();

    // FUNCTIONS //
    void pclCb(const sensor_msgs::PointCloud2ConstPtr& pcl_msg);
    bool newPcl();
    void generateProjectionShapes();
    void publishProjectionShapes();


 private:
    // public ros node handle
    ros::NodeHandle nh_;
    // private ros node handle
    ros::NodeHandle pnh_;
    std::string node_name_{"node_name"};

    // SUBSCRIBERS //
    ros::Subscriber sub_pcl_;

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

    // DYNAMIC RECONFIGURE //
    boost::mutex connect_mutex_;
    boost::recursive_mutex config_mutex_;
    typedef nearness_control::NearnessController3DConfig Config;
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
    double test_ring_;
    bool new_pcl_;

    // GLOBAL VARIABLES //
    float phi_start_;
    float theta_start_;
    float dphi_;
    float dtheta_;
    string frame_id_;

    std::vector<float> phi_view_vec_;
    std::vector<float> theta_view_vec_;

    pcl::PointCloud<pcl::PointXYZ> cloud_out_;
    pcl::PointCloud<pcl::PointXYZ> mu_cloud_out_;
    std::vector<float> mu_sphere_;
    int pcl_width_;
    int pcl_height_;
    int pcl_vertical_spread_;

    pcl::PointCloud<pcl::PointXYZI> Y00_;
    pcl::PointCloud<pcl::PointXYZI> Y0p1_;
    pcl::PointCloud<pcl::PointXYZI> Yp1p1_;
    pcl::PointCloud<pcl::PointXYZI> Yn1p1_;
    pcl::PointCloud<pcl::PointXYZI> Y0p2_;
    pcl::PointCloud<pcl::PointXYZI> Yp1p2_;





}; // class SimpleNodeClass

}  // namespace demo

#endif  // DEMO_SIMPLE_CLASS_NODE_HPP
