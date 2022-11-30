#ifndef NEARNESS_CONTROL_3D_H
#define NEARNESS_CONTROL_3D_H

#include <random>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <nearness_control/projection_shape_generator.h>

// using namespace cv_bridge;
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
  void pclCb(const sensor_msgs::PointCloud2ConstPtr &pcl_msg);

private:
  // Init functions?
  void initRobustController();
  void generateSphericalHarmonics();
  void generateCommandMarkers();

  // pclCb functions
  void processIncomingPCL(const sensor_msgs::PointCloud2ConstPtr &pcl_msg);
  void resetPCLProcessing();
  void projectNearness();
  bool isObstructedPoint(const float t, const float p);
  void resetCommands();
  void resetControllerStates();
  void generateAndPublishCommands();
  void generateStateEstimates();
  void generateWFControlCommands();
  void publishPCLOuts();
  void publishCommandMarkers();

  // Public ros node handle
  ros::NodeHandle nh_;

  // Private ros node handle
  ros::NodeHandle pnh_;
  std::string node_name_{"node_name"};

  // SUBSCRIBERS
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

  // NODE PARAMETERS
  // Enable switches
  bool enable_debug_;
  bool enable_speed_regulation_;
  bool enable_radius_scaling_;
  bool add_noise_;

  // Spherical depth sensor params
  int num_rings_;
  int num_ring_points_;
  int num_basis_shapes_;
  double noise_std_dev_;

  // Forward speed params
  double forward_speed_;
  double max_forward_speed_;
  double min_forward_speed_;
  double k_front_;

  // Lateral speed params
  double k_v_;
  double max_lateral_speed_;

  // Vertical speed params
  double k_w_;
  double max_vertical_speed_;

  // Yaw rate params
  double max_yaw_rate_;
  double k_r_;

  // OTHER VARIABLES
  bool enable_control_ = false;
  std::string frame_id_;

  // Sensor viewing vectors
  float dtheta_, dphi_;
  std::vector<float> phi_view_vec_;
  std::vector<float> theta_view_vec_;
  std::vector<std::vector<float>> viewing_angle_mat_;

  // Helper vars for output debug
  sensor_msgs::PointCloud2 pcl_out_msg_;
  sensor_msgs::PointCloud2 mu_out_msg_;
  pcl::PointCloud<pcl::PointXYZ> new_cloud_;
  pcl::PointCloud<pcl::PointXYZ> cloud_out_;
  pcl::PointCloud<pcl::PointXYZ> mu_cloud_out_;
  std::vector<float> mu_meas_;
  std_msgs::Header pcl_header_;

  int num_excluded_rings_;
  int last_index_;

  nav_msgs::Odometry current_odom_;

  std::vector<float> y_full_;
  std::vector<float> y_front_half_;
  std::vector<float> y_front_half_speed_reg_;
  std::vector<float> y_bottom_half_;
  std_msgs::Float32MultiArray y_projections_msg_;

  std::vector<std::vector<float>> shapes_vec_;

  // Control
  std::vector<float> C_y_;
  std::vector<float> C_z_;
  std::vector<float> C_theta_;
  std::vector<float> u_vec_;
  std::vector<float> state_est_vec_;
  double r_;
  float u_u_, u_v_, u_r_, u_w_;
  geometry_msgs::Twist control_commands_;
  geometry_msgs::Point current_pos_;
  double current_roll_, current_pitch_, current_heading_;
  double current_height_agl_;
  double p_;

  // Control visualization
  visualization_msgs::Marker u_cmd_marker_;
  visualization_msgs::Marker v_cmd_marker_;
  visualization_msgs::Marker w_cmd_marker_;
  visualization_msgs::Marker r_cmd_marker_;
  visualization_msgs::MarkerArray cmd_markers_;

  // Sensor noise
  std::default_random_engine generator_;

  // Dynamic Control
  MatrixXf Mv_A_;
  VectorXf Mv_B_, Mv_C_;
  VectorXf Mv_Xk_, Mv_Xkp1_;

  MatrixXf Mr_A_;
  VectorXf Mr_B_, Mr_C_;
  VectorXf Mr_Xk_, Mr_Xkp1_;

  MatrixXf Mw_A_;
  VectorXf Mw_B_, Mw_C_;
  VectorXf Mw_Xk_, Mw_Xkp1_;

  ros::Time last_pcl_time_;

  // Front speed regulation
  float speed_reg_state_;
  std::vector<float> C_front_;

}; // class

} // namespace nearness_3d

#endif
