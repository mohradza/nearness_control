#include <tf/tf.h>

#include <nearness_control/nearness_control_3d.h>

namespace {
float sgn(double v) { return (v < 0.0) ? -1.0 : ((v > 0.0) ? 1.0 : 0.0); }

float wrapAngle(float angle) {
  if (angle > M_PI) {
    angle -= 2 * M_PI;
  } else if (angle < -M_PI) {
    angle += 2 * M_PI;
  }
  return angle;
}

float sat(float num, float min_val, float max_val) {
  if (num >= max_val) {
    return max_val;
  } else if (num <= min_val) {
    return min_val;
  } else {
    return num;
  }
}
} // namespace

NearnessControl3D::NearnessControl3D(const ros::NodeHandle &node_handle,
                                     const ros::NodeHandle &private_node_handle)
    : nh_(node_handle), pnh_(private_node_handle) {
  this->init();
}

void NearnessControl3D::init() {

  // SUBSCRIBERS
  sub_pcl_ = nh_.subscribe("points", 1, &NearnessControl3D::pclCb, this);
  sub_enable_control_ = nh_.subscribe(
      "enable_control", 1, &NearnessControl3D::enableControlCb, this);

  // PUBLISHERS
  // pcl publishers for debugging
  pub_pcl_ = nh_.advertise<sensor_msgs::PointCloud2>("pcl_out", 1);
  pub_mu_pcl_ = nh_.advertise<sensor_msgs::PointCloud2>("mu_pcl_out", 1);

  // Publish control commands (MAIN OUTPUT)
  pub_control_commands_ =
      nh_.advertise<geometry_msgs::Twist>("control_commands", 1);

  // Cmd markers for visualization
  pub_cmd_markers_ =
      nh_.advertise<visualization_msgs::MarkerArray>("cmd_markers", 1);

  // Import parameters
  pnh_.param("enable_debug", enable_debug_, false);
  pnh_.param("enable_speed_regulation", enable_speed_regulation_, false);
  pnh_.param("enable_radius_scaling", enable_radius_scaling_, false);
  pnh_.param("enable_sensor_noise", add_noise_, false);

  // Sensor params
  pnh_.param("num_rings", num_rings_, 64);
  pnh_.param("num_ring_points", num_ring_points_, 360);
  pnh_.param("num_basis_shapes", num_basis_shapes_, 9);
  pnh_.param("noise_std_dev", noise_std_dev_, 0.03);

  // Forward speed params
  pnh_.param("forward_speed", forward_speed_, .5);
  pnh_.param("forward_speed_max", max_forward_speed_, .5);
  pnh_.param("forward_speed_min", min_forward_speed_, .5);
  pnh_.param("forward_speed_obst_gain", k_front_, .25);

  // Lateral speed params
  pnh_.param("lateral_speed_gain", k_v_, -1.0);
  pnh_.param("lateral_speed_max", max_lateral_speed_, 1.0);

  // Vertical speed params
  pnh_.param("vertical_speed_gain", k_w_, -0.1);
  pnh_.param("vertical_speed_max", max_vertical_speed_, 1.0);

  // Yaw rate Parameters
  pnh_.param("yaw_rate_gain", k_r_, -0.1);
  pnh_.param("yaw_rate_max", max_yaw_rate_, 1.0);

  frame_id_ = "OHRAD_X3";

  // We want to exclude the top and bottom rings
  num_excluded_rings_ = 2;
  last_index_ = (num_rings_ - num_excluded_rings_) * num_ring_points_;

  // Set up the Cdagger matrix
  // Will be referenced as C in code
  C_y_ = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  C_z_ = {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  C_theta_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  C_front_ = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // Initialize dynamic controllers
  initRobustController();

  // Generate the Laplace spherical harmonic basis shapes
  generateSphericalHarmonics();

  // Initialize command markers for debug visualization
  if (enable_debug_) {
    generateCommandMarkers();
  }

} // End of init

void NearnessControl3D::initRobustController() {
  std::vector<double> in_vec;
  std::vector<float> in_vec_f;
  std::vector<double> default_vec;

  // Lateral Controller (Lateral Speed)
  constexpr int Mv_size = 6;
  Mv_Xk_ = Eigen::VectorXf::Zero(Mv_size);
  Mv_Xkp1_ = Mv_Xk_;

  pnh_.param("Mv_A", in_vec, default_vec);
  in_vec_f = std::vector<float>(in_vec.begin(), in_vec.end());
  Mv_A_ = Eigen::Map<Eigen::Matrix<float, Mv_size, Mv_size, RowMajor>>(
      in_vec_f.data());

  pnh_.param("Mv_B", in_vec, default_vec);
  in_vec_f = std::vector<float>(in_vec.begin(), in_vec.end());
  Mv_B_ = Eigen::Map<VectorXf>(in_vec_f.data(), in_vec_f.size());

  pnh_.param("Mv_C", in_vec, default_vec);
  in_vec_f = std::vector<float>(in_vec.begin(), in_vec.end());
  Mv_C_ = Eigen::Map<VectorXf>(in_vec_f.data(), in_vec_f.size());

  // Steering Controller (Heading Rate)
  constexpr int Mr_size = 4;
  Mr_Xk_ = Eigen::VectorXf::Zero(Mr_size);
  Mr_Xkp1_ = Mr_Xk_;

  pnh_.param("Mr_A", in_vec, default_vec);
  in_vec_f = std::vector<float>(in_vec.begin(), in_vec.end());
  Mr_A_ = Eigen::Map<Eigen::Matrix<float, Mr_size, Mr_size, RowMajor>>(
      in_vec_f.data());

  pnh_.param("Mr_B", in_vec, default_vec);
  in_vec_f = std::vector<float>(in_vec.begin(), in_vec.end());
  Mr_B_ = Eigen::Map<VectorXf>(in_vec_f.data(), in_vec_f.size());

  pnh_.param("Mr_C", in_vec, default_vec);
  in_vec_f = std::vector<float>(in_vec.begin(), in_vec.end());
  Mr_C_ = Eigen::Map<VectorXf>(in_vec_f.data(), in_vec_f.size());

  // Vertical Controller (Vertical Speed)
  constexpr int Mw_size = 4;
  Mw_Xk_ = Eigen::VectorXf::Zero(Mw_size);
  Mw_Xkp1_ = Mr_Xk_;

  pnh_.param("Mw_A", in_vec, default_vec);
  in_vec_f = std::vector<float>(in_vec.begin(), in_vec.end());
  Mw_A_ = Eigen::Map<Eigen::Matrix<float, Mw_size, Mw_size, RowMajor>>(
      in_vec_f.data());

  pnh_.param("Mw_B", in_vec, default_vec);
  in_vec_f = std::vector<float>(in_vec.begin(), in_vec.end());
  Mw_B_ = Eigen::Map<VectorXf>(in_vec_f.data(), in_vec_f.size());

  pnh_.param("Mw_C", in_vec, default_vec);
  in_vec_f = std::vector<float>(in_vec.begin(), in_vec.end());
  Mw_C_ = Eigen::Map<VectorXf>(in_vec_f.data(), in_vec_f.size());
}

void NearnessControl3D::generateSphericalHarmonics() {
  ProjectionShapeGenerator generator;
  ProjectionShapeGenerator::ShapeOptions projection_shape_options;
  projection_shape_options.selected_type =
      ProjectionShapeGenerator::ShapeOptions::spherical_harmonics;
  const std::vector<int> shape_size = {num_rings_, num_ring_points_};
  projection_shape_options.size = shape_size;
  shapes_vec_ = shape_generator_.getProjectionShapes(projection_shape_options);
  theta_view_vec_ = shape_generator_.getThetaViewingVector();
  phi_view_vec_ = shape_generator_.getPhiViewingVector();
  viewing_angle_mat_ = shape_generator_.getViewingAngleMatrix();
  dtheta_ = abs(theta_view_vec_[2] - theta_view_vec_[1]);
  dphi_ = abs(phi_view_vec_[2] - phi_view_vec_[1]);
}

void NearnessControl3D::generateCommandMarkers() {
  // Create the command marker origins
  geometry_msgs::Point origin_point;
  origin_point.x = 0.0;
  origin_point.y = 0.0;
  origin_point.z = 0.0;

  geometry_msgs::Quaternion origin_quat;
  origin_quat.x = 0.0;
  origin_quat.y = 0.0;
  origin_quat.z = 0.0;
  origin_quat.w = 1.0;

  u_cmd_marker_.type = 0;
  u_cmd_marker_.id = 0;
  u_cmd_marker_.color.a = 1.0;
  u_cmd_marker_.color.r = 1.0;
  u_cmd_marker_.color.g = 0.0;
  u_cmd_marker_.color.b = 0.0;
  u_cmd_marker_.header.frame_id = "OHRAD_X3";
  u_cmd_marker_.points.push_back(origin_point);
  u_cmd_marker_.scale.x = .05;
  u_cmd_marker_.scale.y = .075;
  u_cmd_marker_.pose.orientation = origin_quat;
  cmd_markers_.markers.push_back(u_cmd_marker_);

  v_cmd_marker_ = u_cmd_marker_;
  v_cmd_marker_.id = 1;
  v_cmd_marker_.color.r = 0.0;
  v_cmd_marker_.color.g = 1.0;
  v_cmd_marker_.color.b = 0.0;
  cmd_markers_.markers.push_back(v_cmd_marker_);

  w_cmd_marker_ = u_cmd_marker_;
  w_cmd_marker_.id = 2;
  w_cmd_marker_.color.r = 0.0;
  w_cmd_marker_.color.b = 1.0;
  cmd_markers_.markers.push_back(w_cmd_marker_);

  r_cmd_marker_ = u_cmd_marker_;
  r_cmd_marker_.id = 3;
  r_cmd_marker_.color.b = 1.0;
  r_cmd_marker_.points[0].x = .5;
  r_cmd_marker_.points[1].x = .5;
  cmd_markers_.markers.push_back(r_cmd_marker_);
}

void NearnessControl3D::enableControlCb(const std_msgs::Bool msg) {
  enable_control_ = msg.data;
}

void NearnessControl3D::pclCb(const sensor_msgs::PointCloud2ConstPtr &pcl_msg) {

  if (enable_control_) {
    processIncomingPCL(pcl_msg);
    projectNearness();
    generateAndPublishCommands();
  } else {
    resetControllerStates();
  }
}

void NearnessControl3D::processIncomingPCL(
    const sensor_msgs::PointCloud2ConstPtr &pcl_msg) {

  // Convert from ROS msg type to PCL pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pcl_msg, *cloud_in);
  const std_msgs::Header pcl_header = pcl_msg->header;
  const size_t pcl_size = cloud_in->points.size();

  // Generate random gaussian noise for the sensor model
  std::normal_distribution<double> noise(0.0, noise_std_dev_);

  pcl::PointXYZ p, mu_p;
  float dist, mu_val;
  int index = 0;

  pcl::PointCloud<pcl::PointXYZ> mu_cloud_out, depth_cloud_out;
  mu_meas_.clear();

  // Process the pointcloud:
  // -- Add Noise
  // -- Convert to nearness
  for (int i = 1; i < num_rings_ - 1; i++) {
    for (int j = 0; j < num_ring_points_; j++) {
      // Rings are positive counterclockwise, so they are reversed on lookup
      index = i * num_ring_points_ + (num_ring_points_ - 1 - j);
      p = cloud_in->points[index];
      dist = sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2));

      if (add_noise_) {
        dist += noise(generator_);
      }

      // Compute nearness
      mu_val = 1.0 / dist;
      mu_meas_.push_back(mu_val);

      if (enable_debug_) {
        // Convert back to cartesian for viewing
        mu_p = {mu_val * sin(theta_view_vec_[i]) * cos(phi_view_vec_[j]),
                mu_val * sin(theta_view_vec_[i]) * sin(phi_view_vec_[j]),
                mu_val * cos(theta_view_vec_[i])};
        mu_cloud_out.push_back(mu_p);
        depth_cloud_out.push_back(p);

        publishPCLOuts(mu_cloud_out, depth_cloud_out, pcl_header);
      }
    } // End inner for loop
  }   // End outer for loop
}

void NearnessControl3D::publishPCLOuts(
    const pcl::PointCloud<pcl::PointXYZ> &mu_cloud,
    const pcl::PointCloud<pcl::PointXYZ> &depth_cloud,
    const std_msgs::Header &pcl_header) {
  sensor_msgs::PointCloud2 depth_pcl_out_msg, mu_pcl_out_msg;

  pcl::toROSMsg(depth_cloud, depth_pcl_out_msg);
  depth_pcl_out_msg.header = pcl_header;
  pub_pcl_.publish(depth_pcl_out_msg);

  pcl::toROSMsg(mu_cloud, mu_pcl_out_msg);
  mu_pcl_out_msg.header = pcl_header;
  pub_mu_pcl_.publish(mu_pcl_out_msg);
}

void NearnessControl3D::publishCommandMarkers(float u_u, float u_v, float u_w,
                                              float u_r) {
  // - Forward speed marker
  ros::Time time_now = ros::Time::now();
  u_cmd_marker_.header.stamp = time_now;
  u_cmd_marker_.points[1].x = u_u;
  cmd_markers_.markers[0] = u_cmd_marker_;

  // - Lateral speed marker
  v_cmd_marker_.header.stamp = time_now;
  v_cmd_marker_.points[1].y = u_v;
  cmd_markers_.markers[1] = v_cmd_marker_;

  // - Forward speed marker
  w_cmd_marker_.header.stamp = time_now;
  w_cmd_marker_.points[1].z = u_w;
  cmd_markers_.markers[2] = w_cmd_marker_;

  // Turn rate marker
  r_cmd_marker_.header.stamp = time_now;
  r_cmd_marker_.points[1].y = u_r;
  cmd_markers_.markers[3] = r_cmd_marker_;

  pub_cmd_markers_.publish(cmd_markers_);
}

bool NearnessControl3D::isObstructedPoint(const float t, const float p) {
  bool obstructed = false;
  constexpr float kThetaUpperLim = 1.768;
  constexpr float kThetaLowerLim = 1.473;
  constexpr float phiLimRotor1Upper = 2.165;
  constexpr float phiLimRotor1Lower = 2.145;
  constexpr float phiLimRotor2Upper = 1.065;
  constexpr float phiLimRotor2Lower = 1.048;

  const bool is_within_inclination_range =
      (t < kThetaUpperLim) && (t > kThetaLowerLim);

  const bool is_rotor_1 = (p < phiLimRotor1Upper) && (p > phiLimRotor1Lower);
  const bool is_rotor_2 = (p < phiLimRotor2Upper) && (p > phiLimRotor2Lower);
  const bool is_rotor_3 = (p < -phiLimRotor2Upper) && (p > -phiLimRotor2Lower);
  const bool is_rotor_4 = (p < -phiLimRotor2Lower) && (p > -phiLimRotor2Upper);
  const bool is_a_rotor =
      (is_rotor_1 || is_rotor_2 || is_rotor_3 || is_rotor_4);
  return is_within_inclination_range && is_a_rotor;
}

void NearnessControl3D::projectNearness() {
  // Project measured nearness onto different shapes
  y_full_.clear();
  y_front_half_.clear();
  y_front_half_speed_reg_.clear();
  float increment;
  float phi, theta;

  for (int j = 0; j < num_basis_shapes_; j++) {
    y_full_.push_back(0.0);
    y_front_half_.push_back(0.0);
    y_front_half_speed_reg_.push_back(0.0);
    for (int i = 0; i < last_index_; i++) {
      // Pull angles out for conveniance
      phi = viewing_angle_mat_[i][1];
      theta = viewing_angle_mat_[i][0];

      // Full projections for centering
      if (!isObstructedPoint(theta, phi)) {
        increment =
            shapes_vec_[j][i] * mu_meas_[i] * sin(theta) * dtheta_ * dphi_;
      } else {
        increment = 0.0;
      }
      y_full_[j] += increment;

      // Front half only for steering
      if (phi < M_PI / 2 && phi > -M_PI / 2) {
        y_front_half_[j] += increment;
      }

      // Front 1/8th only for speed reg
      const bool is_phi_front_zone = (phi < M_PI / 8 && phi > -M_PI / 8);
      const bool is_theta_front_zone =
          (theta > 3.0 * M_PI / 8 && theta < 5.0 * M_PI / 8);
      const bool is_front_zone_sr = is_phi_front_zone && is_theta_front_zone;
      if (is_front_zone_sr) {
        y_front_half_speed_reg_[j] += increment;
      }

    } // End inner for loop
  }   // End outer for loop
}

void NearnessControl3D::generateAndPublishCommands() {

  float u_v, u_r, u_w, u_u;

  // Dynamic lateral controller
  float u_y = k_v_ * state_est_vec_[0];
  Mv_Xkp1_ = Mv_A_ * Mv_Xk_ + Mv_B_ * u_y;
  u_v = Mv_C_.dot(Mv_Xkp1_);
  u_v = sat(u_v, -max_lateral_speed_, max_lateral_speed_);
  Mv_Xk_ = Mv_Xkp1_;

  // Dynamic heading controller
  float u_psi = k_r_ * state_est_vec_[2];
  Mr_Xkp1_ = Mr_A_ * Mr_Xk_ + Mr_B_ * u_psi;
  u_r = Mr_C_.dot(Mr_Xkp1_);
  u_r = sat(u_r, -max_yaw_rate_, max_yaw_rate_);
  Mr_Xk_ = Mr_Xkp1_;

  // Dynamic vertical controller
  float u_z = k_w_ * state_est_vec_[1];
  Mw_Xkp1_ = Mw_A_ * Mw_Xk_ + Mw_B_ * u_z;
  u_w = Mw_C_.dot(Mw_Xkp1_);
  u_w = sat(u_w, -max_vertical_speed_, max_vertical_speed_);
  Mw_Xk_ = Mw_Xkp1_;

  float front_reg;
  if (enable_speed_regulation_) {
    // Need to process the safety zone points for speed regulation
    constexpr float kNominalProjectionVal = 0.07;
    front_reg = (kNominalProjectionVal - speed_reg_state_) * k_front_;
    if (front_reg > 0.0) {
      front_reg = 0.0;
    }
    u_u = sat(forward_speed_ * (1.0 + front_reg), min_forward_speed_,
              max_forward_speed_);
  } else {
    u_u = forward_speed_;
  }

  geometry_msgs::Twist control_commands;
  control_commands.linear.x = u_u;
  control_commands.linear.y = u_v;
  control_commands.linear.z = u_w;
  control_commands.angular.z = u_r;

  pub_control_commands_.publish(control_commands);

  if (enable_debug_) {
    publishCommandMarkers(u_u, u_v, u_w, u_r);
  }
}

void NearnessControl3D::resetControllerStates() {
  constexpr int Mv_size = 6;
  Mv_Xk_ = Eigen::VectorXf::Zero(Mv_size);
  Mv_Xkp1_ = Mv_Xk_;

  constexpr int Mr_size = 4;
  Mr_Xk_ = Eigen::VectorXf::Zero(Mr_size);
  Mr_Xkp1_ = Mr_Xk_;

  constexpr int Mw_size = 4;
  Mw_Xk_ = Eigen::VectorXf::Zero(Mw_size);
  Mw_Xkp1_ = Mr_Xk_;
}

void NearnessControl3D::generateStateEstimates() {
  state_est_vec_ = {0.0, 0.0, 0.0};
  speed_reg_state_ = 0.0;
  for (int j = 0; j < num_basis_shapes_; j++) {
    state_est_vec_[0] += C_y_[j] * y_front_half_[j];
    state_est_vec_[1] += C_z_[j] * y_full_[j];
    state_est_vec_[2] += C_theta_[j] * y_front_half_[j];
    speed_reg_state_ += C_front_[j] * y_front_half_speed_reg_[j];
  }
}

// end of class
