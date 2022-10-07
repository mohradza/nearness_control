#ifndef NEARNESS_CONTROL_3D_H
#define NEARNESS_CONTROL_3D_H

#include <iostream>
#include <random>
#include <ros/ros.h>

#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif
#include <boost/circular_buffer.hpp>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/extract_clusters.h>

#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cstdlib>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <iterator>
#include <math.h>
#include <numeric>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>

#include <Eigen/Core>

#include <boost/thread.hpp>
#include <std_msgs/String.h>

#include <nearness_control/projection_shape_generator.h>

using namespace cv_bridge;
using namespace std;
using namespace Eigen;
namespace nearness_3d {

class NearnessControl3D {
public:
  NearnessControl3D(const ros::NodeHandle &node_handle,
                    const ros::NodeHandle &private_node_handle);
  ~NearnessControl3D() = default;

  void init();

  // FUNCTIONS //
  void enableControlCb(const std_msgs::Bool msg);
  void odomCb(const nav_msgs::OdometryConstPtr &odom_msg);
  void pclCb(const sensor_msgs::PointCloud2ConstPtr &pcl_msg);

private:
  // Init functions
  void initRobustController();
  void generateSphericalHarmonics();
  void generateCommandMarkers();

  // pclCb functions
  void processIncomingPCL(const sensor_msgs::PointCloud2ConstPtr &pcl_msg);
  void resetPCLProcessing();
  void checkFrontZone(const pcl::PointXYZ p);
  void updateZoneStats(const float dist, const int i, const int j);
  bool isSideZonePoint(const float t, const float p);
  bool isVerticalZonePoint(const float t, const float p);
  void updateRadiusEstimates();
  bool isObstructedPoint(const float t, const float p);
  void projectNearness();
  void resetCommands();
  void generateWFControlCommands();
  void publishPCLOuts();
  void publishCommandMarkers();

  // public ros node handle
  ros::NodeHandle nh_;
  // private ros node handle
  ros::NodeHandle pnh_;
  std::string node_name_{"node_name"};

  // SUBSCRIBERS //
  ros::Subscriber sub_pcl_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_enable_control_;

  // PUBLISHERS
  ros::Publisher pub_pcl_;
  ros::Publisher pub_mu_pcl_;

  ros::Publisher pub_y_projection_shape_;
  ros::Publisher pub_theta_projection_shape_;
  ros::Publisher pub_z_projection_shape_;

  ros::Publisher pub_control_commands_;
  ros::Publisher pub_cmd_markers_;

  ProjectionShapeGenerator shape_generator_;

  // GLOBAL VARIABLES //
  bool enable_debug_;
  bool enable_control_ = false;
  bool enable_speed_regulation_;
  bool enable_radius_scaling_;

  string frame_id_;

  float dtheta_, dphi_;
  std::vector<float> phi_view_vec_;
  std::vector<float> theta_view_vec_;
  vector<vector<float>> viewing_angle_mat_;

  sensor_msgs::PointCloud2 pcl_out_msg_;
  sensor_msgs::PointCloud2 mu_out_msg_;

  pcl::PointCloud<pcl::PointXYZ> new_cloud_;
  pcl::PointCloud<pcl::PointXYZ> cloud_out_;
  pcl::PointCloud<pcl::PointXYZ> mu_cloud_out_;
  std::vector<float> mu_meas_;
  std_msgs::Header pcl_header_;

  int num_ring_points_;
  int num_rings_;
  int num_excluded_rings_;
  int num_basis_shapes_;
  int last_index_;

  nav_msgs::Odometry current_odom_;

  vector<float> y_full_;
  vector<float> y_front_half_;
  vector<float> y_bottom_half_;
  std_msgs::Float32MultiArray y_projections_msg_;

  vector<vector<float>> shapes_vec_;

  vector<float> y_projection_shape_vec_;
  vector<float> theta_projection_shape_vec_;
  vector<float> z_projection_shape_vec_;

  // Control
  vector<float> C_y_;
  vector<float> C_z_;
  vector<float> C_theta_;
  vector<float> u_vec_;
  vector<float> state_est_vec_;
  double r_;
  double u_u_, u_v_, u_r_, u_w_;
  double k_v_, k_r_, k_w_;
  geometry_msgs::Twist control_commands_;
  geometry_msgs::Point current_pos_;
  double current_roll_, current_pitch_, current_heading_;
  double current_height_agl_;
  double p_;
  double max_forward_speed_, max_lateral_speed_;
  double max_vertical_speed_, max_yaw_rate_;
  double forward_speed_;

  visualization_msgs::Marker u_cmd_marker_;
  visualization_msgs::Marker v_cmd_marker_;
  visualization_msgs::Marker w_cmd_marker_;
  visualization_msgs::Marker r_cmd_marker_;
  visualization_msgs::MarkerArray cmd_markers_;

  // Sensor noise
  std::default_random_engine generator_;
  double noise_std_dev_;
  bool add_noise_;

  float side_zone_dist_;
  int side_zone_count_;

  float vert_zone_dist_;
  int vert_zone_count_;

  float average_radius_;
  float average_lateral_radius_;
  float average_vertical_radius_;
  float max_dist_;
  float min_dist_;
  // Dynamic Control
  bool enable_dynamic_control_ = false;
  float xv_kp1_, xv_k_, uv_k_;

  Matrix<float, 6, 6> Mv_A_;
  Matrix<float, 6, 1> Mv_B_;
  Matrix<float, 1, 6> Mv_C_;
  Matrix<float, 6, 1> Mv_Xkp1_;
  Matrix<float, 6, 1> Mv_Xk_;

  Matrix<float, 4, 4> Mr_A_;
  Matrix<float, 4, 1> Mr_B_;
  Matrix<float, 1, 4> Mr_C_;
  Matrix<float, 4, 1> Mr_Xkp1_;
  Matrix<float, 4, 1> Mr_Xk_;

  Matrix<float, 4, 4> Mw_A_;
  Matrix<float, 4, 1> Mw_B_;
  Matrix<float, 1, 4> Mw_C_;
  Matrix<float, 4, 1> Mw_Xkp1_;
  Matrix<float, 4, 1> Mw_Xk_;

  Matrix<float, 8, 8> Mc_A_;
  Matrix<float, 8, 2> Mc_B_;
  Matrix<float, 2, 8> Mc_C_;
  Matrix<float, 8, 1> Mc_Xkp1_;
  Matrix<float, 8, 1> Mc_Xk_;

  ros::Time last_pcl_time_;

  // Front speed regulation
  double front_x_lim_, front_y_lim_, front_z_lim_;
  vector<pcl::PointXYZ> safety_zone_points_;
  vector<float> safety_zone_distances_;
  double k_front_;

}; // class

} // namespace nearness_3d

#endif
