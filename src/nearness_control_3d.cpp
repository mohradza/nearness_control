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

std::vector<float> getVectorStats(std::vector<float> vec) {
  double sum = accumulate(begin(vec), end(vec), 0.0);
  double m = sum / vec.size();

  double accum = 0.0;
  std::for_each(begin(vec), end(vec),
                [&](const double d) { accum += (d - m) * (d - m); });

  double stdev = sqrt(accum / (vec.size() - 1));
  std::vector<float> stats{float(m), float(stdev)};
  return stats;
}

} // namespace

namespace nearness_3d {
NearnessControl3D::NearnessControl3D(const ros::NodeHandle &node_handle,
                                     const ros::NodeHandle &private_node_handle)
    : nh_(node_handle), pnh_(private_node_handle) {
  this->init();
}

void NearnessControl3D::init() {

  // SUBSCRIBERS
  sub_pcl_ = nh_.subscribe("points", 1, &NearnessControl3D::pclCb, this);
  sub_odom_ = nh_.subscribe("odometry", 1, &NearnessControl3D::odomCb, this);
  sub_enable_control_ = nh_.subscribe(
      "enable_control", 1, &NearnessControl3D::enableControlCb, this);

  // PUBLISHERS
  // pcl publishers for debugging
  pub_pcl_ = nh_.advertise<sensor_msgs::PointCloud2>("pcl_out", 1);
  pub_mu_pcl_ = nh_.advertise<sensor_msgs::PointCloud2>("mu_pcl_out", 1);
  pub_recon_wf_mu_ =
      nh_.advertise<sensor_msgs::PointCloud2>("reconstructed_wf_nearness", 1);
  pub_sf_mu_ = nh_.advertise<sensor_msgs::PointCloud2>("sf_nearness", 1);

  // Publish control commands (MAIN OUTPUT)
  pub_control_commands_ =
      nh_.advertise<geometry_msgs::Twist>("control_commands", 1);

  // Cmd markers for visualization
  pub_cmd_markers_ =
      nh_.advertise<visualization_msgs::MarkerArray>("cmd_markers", 1);

  // Import parameters
  pnh_.param("enable_debug", enable_debug_, false);
  pnh_.param("enable_sf_control", enable_sf_control_, false);
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
  pnh_.param("forward_speed_obst_gain", k_front_, .25);

  // Lateral speed params
  pnh_.param("lateral_speed_gain", k_v_, -1.0);
  pnh_.param("lateral_speed_max", max_lateral_speed_, 1.0);

  // Vertical speed params
  pnh_.param("vertical_speed_gain", k_w_, -0.1);
  pnh_.param("vertical_speed_max", max_vertical_speed_, 1.0);

  // Yaw rate params
  pnh_.param("yaw_rate_gain", k_r_, -0.1);
  pnh_.param("yaw_rate_max", max_yaw_rate_, 1.0);

  // Small-field conroller params
  pnh_.param("small_field_theta_gain", sf_k_theta_, 1.0);
  pnh_.param("small_field_phi_gain", sf_k_phi_, 1.0);
  pnh_.param("small_field_lateral_distance_gain", sf_k_v_d_, 1.0);
  pnh_.param("small_field_vertical_distance_gain", sf_k_w_d_, 1.0);
  pnh_.param("small_field_lateral_signal_gain", sf_k_v_0_, 1.0);
  pnh_.param("small_field_vertical_signal_gain", sf_k_w_0_, 1.0);

  // Front zone limits
  // TODO: Make these into paramets
  front_x_lim_ = 1.5;
  front_y_lim_ = 0.6;
  front_z_lim_ = 0.25;

  frame_id_ = "OHRAD_X3";

  // We want to exclude the top and bottom rings
  num_excluded_rings_ = 2;
  last_index_ = (num_rings_ - num_excluded_rings_) * num_ring_points_;

  // Set up the Cdagger matrix
  // Will be referenced as C in code
  C_y_ = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  C_z_ = {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  C_theta_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

  // Initialize dynamic controllers
  initRobustController();

  // Generate the Laplace spherical harmonic basis shapes
  generateSphericalHarmonics();

  // Initialize command markers
  generateCommandMarkers();

} // End of init

void NearnessControl3D::initRobustController() {
  Mv_Xk_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  Mr_Xk_ << 0.0, 0.0, 0.0, 0.0;
  Mw_Xk_ << 0.0, 0.0, 0.0, 0.0;
  Mc_Xk_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  // 10hz model
  // Lateral - Mixed synthesis, projection version with new model
  Mv_A_ << 0.9981, 0, 0, 0, 0, 0, 2.0412, 0.0528, -0.2010, -0.2612, -0.4296,
      -7.7748, 6.4777, 0.0270, -0.1694, -1.0669, -1.2556, -24.6354, 5.3906,
      -0.0436, 0.1789, 0.3458, -0.8861, -20.4777, 0.9735, -0.0128, 0.0638,
      0.2929, 0.8476, -3.6970, 0.0036, -0.0001, 0.0003, 0.0021, 0.0119, 0.9862;
  Mv_B_ << 0.0999, 0.5016, 0.6738, 0.2434, 0.0291, 0.0001;
  Mv_C_ << 0.2828, -0.0107, -0.0146, -0.0210, -0.0381, -1.0733;
  Mv_C_ *= 1e3;

  // Heading - Mixed sythensis, projection version with new model
  Mr_A_ << 0.9999, 0.0000, 0.0000, 0.0000, 4.1504, 0.2350, -0.3828, -10.5831,
      3.0972, -0.1106, 0.5083, -7.9036, 0.0187, -0.0008, 0.0072, 0.9524;
  Mr_B_ << 0.0200, 0.0582, 0.0373, 0.0001;
  Mr_C_ << 139.2399, -8.7243, -12.4138, -355.0898;

  // Vertical - Mixed sythesis, 2x2 state model, 1 input, no modifications
  Mw_A_ << 0.9996, 0.0000, 0.0000, 0.0000, 1.7728, 0.1060, -0.1395, -3.3989,
      6.3121, -0.3233, 0.3998, -12.1131, 0.0550, -0.0053, 0.0078, 0.8945;
  Mw_B_ << 0.1000, 0.2087, 0.4397, 0.0022;
  Mw_C_ << 78.6214, -17.2273, -4.9615, -150.7364;
}

void NearnessControl3D::generateSphericalHarmonics() {
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

void NearnessControl3D::odomCb(const nav_msgs::OdometryConstPtr &odom_msg) {
  current_odom_ = *odom_msg;
  current_pos_ = current_odom_.pose.pose.position;
  // current_odom_.pose.pose.position.z = current_height_agl_;
  geometry_msgs::Quaternion vehicle_quat_msg =
      current_odom_.pose.pose.orientation;
  tf::Quaternion vehicle_quat_tf;
  tf::quaternionMsgToTF(vehicle_quat_msg, vehicle_quat_tf);
  tf::Matrix3x3(vehicle_quat_tf)
      .getRPY(current_roll_, current_pitch_, current_heading_);
  p_ = current_odom_.twist.twist.angular.x;
  r_ = current_odom_.twist.twist.angular.z;
}

void NearnessControl3D::pclCb(const sensor_msgs::PointCloud2ConstPtr &pcl_msg) {

  processIncomingPCL(pcl_msg);

  projectNearness();

  // Compute control commands
  resetCommands();
  if (enable_control_) {
    generateWFControlCommands();
    if (enable_sf_control_) {
      generateSFControlCommands();
    }
    mixControlCommands();
    pub_control_commands_.publish(control_commands_);
  }

  if (enable_debug_) {
    // Publish pcl outs
    publishPCLOuts();
    // Publish control command markers for visualization
    publishCommandMarkers();
  }
}

void NearnessControl3D::processIncomingPCL(
    const sensor_msgs::PointCloud2ConstPtr &pcl_msg) {

  // Convert from ROS msg type to PCL pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pcl_msg, *cloud_in);
  size_t pcl_size = cloud_in->points.size();
  new_cloud_ = *cloud_in;
  pcl_header_ = pcl_msg->header;

  // Reset counts and other vectors
  resetPCLProcessing();

  // Generate random gaussian noise for the sensor model
  std::normal_distribution<double> noise(0.0, noise_std_dev_);

  // Exclude the first and last rings, because they don't contain much
  // information

  pcl::PointXYZ p, mu_p;
  float dist, mu_val;
  int index = 0;

  // Process the pointcloud:
  // -- Add Noise
  // -- Convert to nearness
  // -- Process front and side zone measurements
  for (int i = 1; i < num_rings_ - 1; i++) {
    for (int j = 0; j < num_ring_points_; j++) {
      // Rings are positive counterclockwise, so they are reversed on lookup
      index = i * num_ring_points_ + (num_ring_points_ - 1 - j);
      p = new_cloud_.points[index];
      distance_scan_pcl_.push_back(p);
      dist = sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2));

      if (dist < 2.0 && dist > 0.27) {
        checkFrontZone(p);
      }

      if (add_noise_) {
        dist += noise(generator_);
      }

      // This is within the vehicle frame, remove (set to large value)
      if (dist < 0.3) {
        dist = 1000;
      }

      // Compute nearness
      mu_val = 1.0 / dist;
      mu_meas_.push_back(mu_val);

      if (enable_radius_scaling_) {
        updateZoneStats(dist, i, j);
      }

      if (enable_debug_) {
        // Convert back to cartesian for viewing
        mu_p = {mu_val * sin(theta_view_vec_[i]) * cos(phi_view_vec_[j]),
                mu_val * sin(theta_view_vec_[i]) * sin(phi_view_vec_[j]),
                mu_val * cos(theta_view_vec_[i])};
        mu_cloud_out_.push_back(mu_p);
        cloud_out_.push_back(p);
      }
    } // End inner for loop
  }   // End outer for loop

  if (enable_radius_scaling_) {
    updateRadiusEstimates();
  }
}

void NearnessControl3D::resetPCLProcessing() {
  mu_meas_.clear();
  cloud_out_.clear();
  mu_cloud_out_.clear();
  safety_zone_points_.clear();
  safety_zone_distances_.clear();

  side_zone_dist_ = 0.0;
  side_zone_count_ = 0;
  vert_zone_dist_ = 0.0;
  vert_zone_count_ = 0;
}

void NearnessControl3D::checkFrontZone(const pcl::PointXYZ p) {
  // If the point exists inside our front safety zone, add it to
  // the collection of current safety zone violation points
  if (p.z < front_z_lim_ && p.z > -front_z_lim_) {
    if (p.x > 0.0 && p.x < front_x_lim_) {
      if (p.y < front_y_lim_ && p.y > -front_y_lim_) {
        // The point violates the safety zone
        // Need to consider it for speed regulation.
        safety_zone_points_.push_back(p);

        // Really we just need the raw distance
        safety_zone_distances_.push_back(
            sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2)));
      }
    }
  }
}

void NearnessControl3D::updateZoneStats(const float dist, const int i,
                                        const int j) {
  float distance = dist;
  if (distance > 5.0) {
    distance = 5.0;
  }
  if (isSideZonePoint(theta_view_vec_[i], phi_view_vec_[j])) {
    side_zone_dist_ += distance;
    side_zone_count_ += 1;
  }
  if (isVerticalZonePoint(theta_view_vec_[i], phi_view_vec_[j])) {
    vert_zone_dist_ += distance;
    vert_zone_count_ += 1;
  }
}

void NearnessControl3D::updateRadiusEstimates() {
  // TODO: Make these into parameters!!
  average_lateral_radius_ = 2.7;
  average_vertical_radius_ = 1.75;
  // Compute the average radius
  if (side_zone_count_ && vert_zone_count_) {
    average_lateral_radius_ = side_zone_dist_ / side_zone_count_;
    average_vertical_radius_ = vert_zone_dist_ / vert_zone_count_;
    ROS_INFO_THROTTLE(2.0, "Side: %f, Vert: %f", average_lateral_radius_,
                      average_vertical_radius_);
  } else {
    ROS_INFO_THROTTLE(1.0, "Missing zone counts: side: %d, vert: %d",
                      side_zone_count_, vert_zone_count_);
  }
}

void NearnessControl3D::publishPCLOuts() {
  pcl::toROSMsg(cloud_out_, pcl_out_msg_);
  pcl_out_msg_.header = pcl_header_;
  pub_pcl_.publish(pcl_out_msg_);

  pcl::toROSMsg(mu_cloud_out_, mu_out_msg_);
  mu_out_msg_.header = pcl_header_;
  pub_mu_pcl_.publish(mu_out_msg_);
}

void NearnessControl3D::publishCommandMarkers() {
  // - Forward speed marker
  ros::Time time_now = ros::Time::now();
  u_cmd_marker_.header.stamp = time_now;
  u_cmd_marker_.points[1].x = u_u_;
  cmd_markers_.markers[0] = u_cmd_marker_;

  // - Lateral speed marker
  v_cmd_marker_.header.stamp = time_now;
  v_cmd_marker_.points[1].y = u_v_;
  cmd_markers_.markers[1] = v_cmd_marker_;

  // - Forward speed marker
  w_cmd_marker_.header.stamp = time_now;
  w_cmd_marker_.points[1].z = u_w_;
  cmd_markers_.markers[2] = w_cmd_marker_;

  // Turn rate marker
  r_cmd_marker_.header.stamp = time_now;
  r_cmd_marker_.points[1].y = u_r_;
  cmd_markers_.markers[3] = r_cmd_marker_;

  pub_cmd_markers_.publish(cmd_markers_);
}

bool NearnessControl3D::isSideZonePoint(const float t, const float p) {
  float side_delta_p = M_PI / 16.0;
  float side_delta_t = M_PI / 32.0;
  // Point is in the left side zone
  if (p < (-M_PI / 2.0 + side_delta_p) && p > (-M_PI / 2.0 - side_delta_p)) {
    if (t > (M_PI / 2.0 - side_delta_t) && t < (M_PI / 2.0 + side_delta_t)) {
      return true;
    }
  }
  // Point is in the right side zone
  if (p > (M_PI / 2.0 - side_delta_p) && p < (M_PI / 2.0 + side_delta_p)) {
    if (t > (M_PI / 2.0 - side_delta_t) && t < (M_PI / 2.0 + side_delta_t)) {
      return true;
    }
  }

  return false;
}

bool NearnessControl3D::isVerticalZonePoint(const float t, const float p) {
  float vert_delta_t = M_PI / 8.0;
  // Point is in the top or bottom zone
  if (t < (vert_delta_t) || t > (M_PI - vert_delta_t)) {
    return true;
  }

  return false;
}

bool NearnessControl3D::isObstructedPoint(const float t, const float p) {
  bool obstructed = false;

  if (t < 1.768 && t > 1.473) {
    if ((p < 2.165 && p > 2.145) || (p < 1.065 && p > 1.046) ||
        (p < -1.028 && p > -1.048) || (p < -2.145 && p > -2.165)) {
      obstructed = true;
    }
  }
  return obstructed;
}

void NearnessControl3D::projectNearness() {
  // Project measured nearness onto different shapes
  y_full_.clear();
  y_front_half_.clear();
  float increment;
  float phi, theta;

  for (int j = 0; j < num_basis_shapes_; j++) {
    y_full_.push_back(0.0);
    y_front_half_.push_back(0.0);
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

    } // End inner for loop
  }   // End outer for loop
}

void NearnessControl3D::resetCommands() {
  control_commands_.linear.x = 0.0;
  control_commands_.linear.y = 0.0;
  control_commands_.linear.z = 0.0;

  control_commands_.angular.x = 0.0;
  control_commands_.angular.y = 0.0;
  control_commands_.angular.z = 0.0;

  u_u_ = 0.0;
  u_w_ = 0.0;
  u_v_ = 0.0;
  u_r_ = 0.0;
}

void NearnessControl3D::generateWFControlCommands() {
  // ros::Time now1 = ros::Time::now();
  u_vec_ = {0.0, 0.0, 0.0};
  state_est_vec_ = {0.0, 0.0, 0.0};

  for (int j = 0; j < num_basis_shapes_; j++) {
    state_est_vec_[0] += C_y_[j] * y_full_[j];
    state_est_vec_[1] += C_z_[j] * y_full_[j];
    state_est_vec_[2] += C_theta_[j] * y_front_half_[j];
  }

  // Complex lateral dynamic controller
  float u_y = k_v_ * state_est_vec_[0];
  Mv_Xkp1_ = Mv_A_ * Mv_Xk_ + Mv_B_ * u_y;
  u_v_ = Mv_C_ * Mv_Xkp1_;
  u_v_ = sat(u_v_, -max_lateral_speed_, max_lateral_speed_);
  Mv_Xk_ = Mv_Xkp1_;
  //
  // // Complex heading dynamic controller
  float u_psi = k_r_ * state_est_vec_[2];
  Mr_Xkp1_ = Mr_A_ * Mr_Xk_ + Mr_B_ * u_psi;
  u_r_ = Mr_C_ * Mr_Xkp1_;
  u_r_ = sat(u_r_, -max_yaw_rate_, max_yaw_rate_);
  Mr_Xk_ = Mr_Xkp1_;

  // Complex vertical dynamic controller
  float u_z = k_w_ * state_est_vec_[1];
  Mw_Xkp1_ = Mw_A_ * Mw_Xk_ + Mw_B_ * u_z;
  u_w_ = Mw_C_ * Mw_Xkp1_;
  u_w_ = sat(u_w_, -max_vertical_speed_, max_vertical_speed_);
  Mw_Xk_ = Mw_Xkp1_;

  if (enable_radius_scaling_) {
    u_v_ *= average_lateral_radius_ / 2.7;
    u_r_ *= average_lateral_radius_ / 2.7;
    u_w_ *= average_vertical_radius_ / 1.75;
  }
}

void NearnessControl3D::generateSFControlCommands() {
  reconstructWFNearness();
  generateSFNearness();
  computeSFControl();
}

void NearnessControl3D::reconstructWFNearness() {

  recon_wf_mu_vec_.clear();
  std::vector<float> zeros(last_index_, 0.0);
  recon_wf_mu_vec_ = zeros;
  for (int j = 0; j < num_wf_harmonics_; j++) {
    for (int i = 0; i < last_index_; i++) {
      recon_wf_mu_vec_[i] += y_full_[j] * shapes_vec_[j][i];
    }
  }

  pcl::PointXYZ recon_mu_p;
  recon_wf_mu_pcl_.clear();

  if (enable_debug_) {
    // Turn reconstructed wf back into pointcloud for viewing
    float theta, phi;
    for (int i = 0; i < last_index_; i++) {
      theta = viewing_angle_mat_[i][0];
      phi = viewing_angle_mat_[i][1];

      recon_mu_p = {recon_wf_mu_vec_[i] * sin(theta) * cos(phi),
                    recon_wf_mu_vec_[i] * sin(theta) * sin(phi),
                    recon_wf_mu_vec_[i] * cos(theta)};
      recon_wf_mu_pcl_.push_back(recon_mu_p);
    }

    pcl::toROSMsg(recon_wf_mu_pcl_, recon_wf_mu_pcl_msg_);
    recon_wf_mu_pcl_msg_.header.frame_id = frame_id_;
    recon_wf_mu_pcl_msg_.header.stamp = ros::Time::now();
    pub_recon_wf_mu_.publish(recon_wf_mu_pcl_msg_);
  }
}

void NearnessControl3D::generateSFNearness() {

  sf_mu_.clear();
  sf_mu_pcl_.clear();
  float diff, theta, phi;
  pcl::PointXYZ sf_mu_p, sf_d_p;
  for (int i = 0; i < last_index_; i++) {
    diff = mu_meas_[i] - recon_wf_mu_vec_[i];
    sf_mu_.push_back(diff);
    sf_diff_index_.push_back(i);
    theta = viewing_angle_mat_[i][0];
    phi = viewing_angle_mat_[i][1];
    diff = abs(diff);
    sf_mu_p = {diff * sin(theta) * cos(phi), diff * sin(theta) * sin(phi),
               diff * cos(theta)};
    sf_mu_pcl_.push_back(sf_mu_p);
  }

  if (enable_debug_) {
    pcl::toROSMsg(sf_mu_pcl_, sf_mu_pcl_msg_);
    sf_mu_pcl_msg_.header.frame_id = frame_id_;
    sf_mu_pcl_msg_.header.stamp = ros::Time::now();
    pub_sf_mu_.publish(sf_mu_pcl_msg_);
  }
}

void NearnessControl3D::thresholdAndClusterSFNearness() {
  // Iterate through SF Pcl and get stats for dynamic threshold
  std::vector<float> sf_stats = getVectorStats(sf_mu_);

  // Create a dynamic threshold for mu
  float dyn_thresh = 3.0 * sf_stats[1];

  // Remove any point that is below the threshold. It is easier to deal with
  // distance from this point out... so we will only collect filtered distance
  sf_d_pcl_filtered_.clear();
  std::vector<std::vector<float>> viewing_angle_mat_filtered;
  std::vector<float> sf_d_filtered;
  for (int i = 0; i < last_index_; i++) {
    if (sf_mu_[i] >= dyn_thresh) {
      sf_d_pcl_filtered_.push_back(distance_scan_pcl_[i]);
      sf_d_filtered.push_back(1 / mu_meas_[i]);
      viewing_angle_mat_filtered.push_back(viewing_angle_mat_[i]);
    }
  }

  // Cluster the peaks using Euclidean cluster segmentation
  pcl::PointCloud<pcl::PointXYZ> *xyz_cloud_filtered;
  *xyz_cloud_filtered = sf_d_pcl_filtered_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtrFiltered(xyz_cloud_filtered);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(xyzCloudPtrFiltered);

  std::vector<pcl::PointIndices> cluster_indices;

  // Specify cluster parameters
  ece_.setClusterTolerance(10); // cm
  ece_.setMinClusterSize(5);    // Should help with noise
  ece_.setMaxClusterSize(last_index_);
  ece_.setSearchMethod(tree);
  ece_.setInputCloud(xyzCloudPtrFiltered);
  // Extract and store indices
  ece_.extract(cluster_indices);

  // Determine cluster centroid and distance
  num_clusters_ = cluster_indices.size();
  int index = 0;
  float theta_ave, phi_ave, d_ave;
  int num_cluster_points;
  cluster_locs_.clear();
  cluster_d_.clear();
  for (int i = 0; i < num_clusters_; i++) {
    theta_ave = 0.0;
    phi_ave = 0.0;
    num_cluster_points = cluster_indices[i].indices.size();
    for (int j = 0; j < num_clusters_; j++) {
      index = cluster_indices[i].indices[j];
      theta_ave += viewing_angle_mat_filtered[index][0];
      phi_ave += viewing_angle_mat_filtered[index][1];
      d_ave += sf_d_filtered[index];
    }
    cluster_locs_.push_back(
        {theta_ave / num_cluster_points, phi_ave / num_cluster_points});
    cluster_d_.push_back(d_ave / num_cluster_points);
  }
}

void NearnessControl3D::computeSFControl() {
  // Generate control commands: lateral speed, vertical speed
  float vert_sign, horiz_sign, cluster_theta, cluster_phi;
  sf_u_w_ = 0.0;
  sf_u_v_ = 0.0;
  for (int i = 0; i < num_clusters_; i++) {
    vert_sign = sgn((M_PI / 2 - cluster_theta));
    horiz_sign = sgn(cluster_phi);
    cluster_theta = cluster_locs_[i][0];
    cluster_phi = cluster_locs_[i][1];

    sf_u_v_ += sf_k_v_0_ * vert_sign *
               exp(-sf_k_phi_ * abs(M_PI / 2 - cluster_phi)) *
               exp(-sf_k_v_d_ / cluster_d_[i]);
    sf_u_w_ += sf_k_w_0_ * vert_sign *
               exp(-sf_k_theta_ * abs(M_PI / 2 - cluster_theta)) *
               exp(-sf_k_w_d_ / cluster_d_[i]);
  }
}

void NearnessControl3D::mixControlCommands() {
  float front_reg;
  if (enable_speed_regulation_) {
    // Need to process the safety zone points for speed regulation
    if (safety_zone_distances_.size()) {
      // Find the closest point and use that for speed reg
      float min_val = *min_element(safety_zone_distances_.begin(),
                                   safety_zone_distances_.end());
      front_reg = k_front_ * (1 / min_val);
      u_u_ = sat(forward_speed_ * (1.0 - front_reg), -0.5, max_forward_speed_);
    } else {
      u_u_ = forward_speed_;
    }
  } else {
    u_u_ = forward_speed_;
  }

  if (enable_sf_control_) {
    u_v_ += sf_u_v_;
    u_w_ += sf_u_w_;
    ROS_INFO_THROTTLE(0.25, "sf_u_v: %f, sf_u_w: %f", sf_u_v_, sf_u_w_);
  }

  control_commands_.linear.x = u_u_;
  control_commands_.linear.y = u_v_;
  control_commands_.linear.z = u_w_;
  control_commands_.angular.z = u_r_;
  ROS_INFO_THROTTLE(0.25, "u_u: %f, u_v: %f, u_r: %f, u_w: %f", u_u_, u_v_,
                    u_r_, u_w_);
}
} // namespace nearness_3d
