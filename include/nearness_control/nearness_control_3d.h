#ifndef NEARNESS_CONTROL_3D_H
#define NEARNESS_CONTROL_3D_H

#include <memory>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/Twist.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <nearness_control/projection_shape_generator.h>

// using namespace cv_bridge;
using namespace Eigen;

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
  // Init functions
  void initRobustController();
  void generateSphericalHarmonics();
  void generateCommandMarkers();

  // pclCb functions
  void processIncomingPCL(const sensor_msgs::PointCloud2ConstPtr &pcl_msg);

  // Project measured nearness onto the different basis shapes
  void projectNearness();

  // Check to see if the measured point belongs to the set of points obstructed
  // by the rotors of the quadrotor.
  bool isObstructedPoint(const float t, const float p);

  void resetControllerStates();

  // Pool projections to generate the environment relative state estimates
  void generateStateEstimates();

  // Use the state estimates from wide-field integration to create outer-loop
  // control commands using a robust, dynamic, h-inifity controller
  void generateAndPublishCommands();

  // Debug functions for displaying useful data
  void publishPCLOuts(const pcl::PointCloud<pcl::PointXYZ> &mu_pcl,
                      const pcl::PointCloud<pcl::PointXYZ> &depth_pc,
                      const std_msgs::Header &pcl_header);
  void publishCommandMarkers(float u_u, float u_v, float u_w, float u_r);

  // Public ros node handle
  ros::NodeHandle nh_;

  // Private ros node handle
  ros::NodeHandle pnh_;
  std::string node_name_{"node_name"};

  // SUBSCRIBERS
  ros::Subscriber sub_pcl_;
  ros::Subscriber sub_enable_control_;

  // PUBLISHERS
  ros::Publisher pub_pcl_;
  ros::Publisher pub_mu_pcl_;

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
  std::vector<float> phi_view_vec_, theta_view_vec_;
  std::vector<std::vector<float>> viewing_angle_mat_;

  std::vector<float> mu_meas_;

  int num_excluded_rings_;
  int last_index_;

  std::vector<float> y_full_;
  std::vector<float> y_front_half_;
  std::vector<float> y_front_half_speed_reg_;

  std::vector<std::vector<float>> shapes_vec_;

  // Control
  std::vector<float> C_y_, C_z_, C_theta_;
  std::vector<float> state_est_vec_;

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

#endif
