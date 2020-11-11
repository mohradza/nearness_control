#include <nearness_control/nearness_controller_3d.h>

namespace nearness_3d{
NearnessController3D::NearnessController3D(const ros::NodeHandle &node_handle,
                                       const ros::NodeHandle &private_node_handle)
      :nh_(node_handle),
       pnh_(private_node_handle) {
      this->init();
  }

void NearnessController3D::init() {
    // Set up dynamic reconfigure
    reconfigure_server_.reset(new ReconfigureServer(config_mutex_, pnh_));
    ReconfigureServer::CallbackType f = boost::bind(&NearnessController3D::configCb, this, _1, _2);
    reconfigure_server_->setCallback(f);

    // Set up subscribers and callbacks
    sub_pcl_ = nh_.subscribe("points", 1, &NearnessController3D::pclCb, this);
    sub_odom_ = nh_.subscribe("odometry", 1, &NearnessController3D::odomCb, this);
    sub_joy_ = nh_.subscribe("joy", 1, &NearnessController3D::joyconCb, this);

    // Set up publishers
    pub_pcl_ = nh_.advertise<sensor_msgs::PointCloud2>("pcl_out",1);
    pub_mu_pcl_ = nh_.advertise<sensor_msgs::PointCloud2>("mu_pcl_out",1);
    pub_mu_pcl2_ = nh_.advertise<sensor_msgs::PointCloud2>("mu_pcl_test_out",1);
    pub_dist_pcl_ = nh_.advertise<sensor_msgs::PointCloud2>("dist_test_out",1);
    pub_Y00_ = nh_.advertise<sensor_msgs::PointCloud2>("Y00",1);
    pub_Y0p1_ = nh_.advertise<sensor_msgs::PointCloud2>("Y0p1",1);
    pub_Yp1p1_ = nh_.advertise<sensor_msgs::PointCloud2>("Yp1p1",1);
    pub_Yn1p1_ = nh_.advertise<sensor_msgs::PointCloud2>("Yn1p1",1);
    pub_Y0p2_ = nh_.advertise<sensor_msgs::PointCloud2>("Y0p2",1);
    pub_Yp1p2_ = nh_.advertise<sensor_msgs::PointCloud2>("Yp1p2",1);
    pub_Yn1p2_ = nh_.advertise<sensor_msgs::PointCloud2>("Yn1p2",1);
    pub_Yp2p2_ = nh_.advertise<sensor_msgs::PointCloud2>("Yp2p2",1);
    pub_Yn2p2_ = nh_.advertise<sensor_msgs::PointCloud2>("Yn2p2",1);
    pub_y_projection_shape_ = nh_.advertise<sensor_msgs::PointCloud2>("y_projection_shape",1);
    pub_theta_projection_shape_ = nh_.advertise<sensor_msgs::PointCloud2>("theta_projection_shape",1);
    pub_z_projection_shape_ = nh_.advertise<sensor_msgs::PointCloud2>("z_projection_shape",1);
    pub_y_projections_with_odom_ = nh_.advertise<nearness_control_msgs::ProjectionWithOdomMsg>("y_projections_with_odom",1);
    pub_recon_wf_mu_ = nh_.advertise<sensor_msgs::PointCloud2>("reconstructed_wf_nearness",1);
    pub_sf_mu_ = nh_.advertise<sensor_msgs::PointCloud2>("sf_nearness",1);
    pub_control_commands_ = nh_.advertise<geometry_msgs::Twist>("control_commands",1);
    pub_cmd_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("cmd_markers",1);

    // Import parameters
    pnh_.param("enable_debug", enable_debug_, false);
    pnh_.param("enable_altitude_hold", enable_altitude_hold_, false);
    pnh_.param("enable_speed_regulation", enable_speed_regulation_, false);

    pnh_.param("num_rings", num_rings_, 64);
    pnh_.param("num_ring_points", num_ring_points_, 360);
    pnh_.param("num_basis_shapes", num_basis_shapes_, 9);
    pnh_.param("num_wf_harmonics", num_wf_harmonics_, 9);

    pnh_.param("lateral_speed_gain", k_v_, -0.1);
    pnh_.param("turn_rate_gain", k_thetadot_, -0.1);
    pnh_.param("vertical_speed_gain", k_w_, -0.1);
    pnh_.param("forward_speed", forward_speed_, .5);
    pnh_.param("reference_altitude", reference_altitude_, .5);
    pnh_.param("forward_speed_lateral_gain", k_u_v_, .1);
    pnh_.param("forward_speed_thetadot_gain", k_u_thetadot_, .1);

    max_forward_speed_ = 1.0;
    max_lateral_speed_ = 1.0;
    max_vertical_speed_ = 1.0;
    max_yaw_rate_ = 1.0;

    ROS_INFO("%f", k_thetadot_);

    enable_control_ = false;
    enable_cmd_scaling_ = true;

    frame_id_ = "OHRAD_X3";

    // We want to exclude the top and bottom rings
    num_excluded_rings_ = 2;
    // if(half_projections_){
    //   num_ring_points_ = num_ring_points_/2;
    // }
    last_index_ = (num_rings_- num_excluded_rings_)*num_ring_points_;

    new_pcl_ = false;
    y_projections_with_odom_msg_.num_projections = num_basis_shapes_;

    // Set up the Cdagger matrix
    // Will be referenced as C in code
    // Might want to implement this with Eigen
    vector<float> zeros_vec(num_basis_shapes_, 0.0);
    //C_dy_ = zeros_vec;
    //C_dy_ = {0.0137, -0.2525, 0.1475, -3.7765, 0.0200, -0.2169, -1.1179, 0.3504, -.0037};
    C_dy_ = {-0.4665, 0.0037, 0.2523, 3.3906, 1.6712, 0.0114, 1.8488, 0.0045, -0.0378};
    //C_mat_.push_back(C_dy_);
    //C_dtheta_ = zeros_vec;
    //C_dtheta_ = {-0.0408, 0.0168, -0.1982, -0.0513, 0.1610, -0.0519, 0.0344, 0.0596, -2.6322};
    //C_dtheta_ = {0.0820, -0.1909, -0.6301, 0.0640, -0.3424, -0.0814, -0.1268, 0.0088, 2.6603};
    // Half Front sphere
    C_dtheta_ = {-0.2357, -0.0990, 0.2396, 0.0491, 0.7532, -0.0251, -0.2976, -0.1963, 5.3746};

  //  C_mat_.push_back(C_dtheta_);

    //C_dz_ = zeros_vec;
    // Full Sphere
    C_dz_ = {0.1425, -1.2913, -1.6798, 2.6784, 0.1223, -0.0236, -6.7826, 2.0289, -2.9310};
    // Half Sphere
    //C_z_ = {0.0006589,  -0.0002733, -0.0005839, 0.00004139, -0.001567, -0.0001882, 0.0009429, -0.0009308, 0.002691};
    //C_mat_.push_back(C_dz_);

    // Prepare the Laplace spherical harmonic basis set
    generateViewingAngleVectors();
    generateProjectionShapes();

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

    thetadot_cmd_marker_ = u_cmd_marker_;
    thetadot_cmd_marker_.id = 3;
    thetadot_cmd_marker_.color.b = 1.0;
    thetadot_cmd_marker_.points[0].x = .5;
    thetadot_cmd_marker_.points[1].x = .5;
    cmd_markers_.markers.push_back(thetadot_cmd_marker_);


} // End of init

void NearnessController3D::configCb(Config &config, uint32_t level)
{
    config_ = config;

    // Controller gains
    //u_k_hb_1_ = config_.forward_speed_k_hb_1;
    //u_k_hb_2_ = config_.forward_speed_k_hb_2;

}

bool NearnessController3D::newPcl(){
  return new_pcl_;
}

void NearnessController3D::generateViewingAngleVectors(){

  // Inclination angle
  // TODO: These should also be parameters
  float theta_start = M_PI;
  float theta_end = 0;
  dtheta_ = (theta_start - theta_end)/float(num_rings_);
  for(int i = 0; i <= num_rings_; i++){
      theta_view_vec_.push_back(theta_start - float(i)*dtheta_);
  }

  // Azimuthal angle
  // TODO: These should also be parameters
  float phi_start = M_PI;
  float phi_end = -M_PI;
  dphi_ = (phi_start - phi_end)/float(num_ring_points_);
  for(int i = 0; i < num_ring_points_; i++){
    phi_view_vec_.push_back(phi_start - float(i)*dphi_);
  }

  // Make a matrix for making generating pointclouds from
  // vector representations of nearness
  for(int i = 1; i < num_rings_-1; i++){
    for(int j = 0; j < num_ring_points_; j++){
      viewing_angle_mat_.push_back({theta_view_vec_[i], phi_view_vec_[j]});
    }
  }

}

void NearnessController3D::pclCb(const sensor_msgs::PointCloud2ConstPtr& pcl_msg){

  new_pcl_ = true;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*pcl_msg, *cloud_in);
  size_t pcl_size = cloud_in->points.size();

  mu_meas_.clear();
  cloud_out_.clear();
  mu_cloud_out_.clear();

  int count = 0;
  int ring_win_size = 10;
  int scan_win_size = 2;
  front_mu_ave_ = 0.0;

  // Convert the pcl to nearness
  pcl::PointXYZ p, mu_p;
  float dist, mu_val;
  for(int i = 1; i < num_rings_-1; i++){
      for(int j = 0; j < num_ring_points_; j++){
          // Rings are positive counterclockwise from sensor
          // So they are reversed on lookup
          p = cloud_in->points[i*num_ring_points_ + (num_ring_points_-1-j)];
          cloud_out_.push_back(p);
          dist = sqrt(pow(p.x,2) + pow(p.y,2) + pow(p.z,2));
          mu_val = 1/dist;
          mu_meas_.push_back(mu_val);

          // Get an estimate of how close things are in front of the vehicle
          // to use for speed regulation. Look at a window of nearness.
          if((j < (num_ring_points_/2 + ring_win_size)) && (j > (num_ring_points_/2 - ring_win_size)) && (i < (num_rings_/2 + scan_win_size)) && (i > (num_rings_/2 - scan_win_size))){
            front_mu_ave_ += mu_val;
            count++;
          }

          // Convert back to cartesian for viewing
          if(enable_debug_){
              mu_p = {mu_val*sin(theta_view_vec_[i])*cos(phi_view_vec_[j]), mu_val*sin(theta_view_vec_[i])*sin(phi_view_vec_[j]), mu_val*cos(theta_view_vec_[i]) };
              mu_cloud_out_.push_back(mu_p);
          }
      }
  }

  front_mu_ave_ /= count;

  // Treat the first point as a rangefinder for alt hold
  p = cloud_in->points[0];
  dist = sqrt(pow(p.x,2) + pow(p.y,2) + pow(p.z,2));
  current_height_agl_ = dist*cos(current_roll_)*cos(current_pitch_);

  if(enable_debug_){
      pcl::toROSMsg(cloud_out_, pcl_out_msg_);
      pcl_out_msg_.header = pcl_msg->header;
      pub_pcl_.publish(pcl_out_msg_);

      pcl::toROSMsg(mu_cloud_out_, mu_out_msg_);
      mu_out_msg_.header = pcl_msg->header;
      pub_mu_pcl_.publish(mu_out_msg_);
  }

}

void NearnessController3D::projectNearness(){

  // Project measured nearness onto different shapes
  y_full_.clear();
  y_front_half_.clear();
  y_bottom_half_.clear();

  float phi, theta, increment;
  for(int j = 0; j < num_basis_shapes_; j++){
    y_full_.push_back(0.0);
    y_front_half_.push_back(0.0);
    y_bottom_half_.push_back(0.0);
    for (int i = 0; i < last_index_; i++){

      phi = viewing_angle_mat_[i][1];
      theta = viewing_angle_mat_[i][0];

      // Full projections for centering
      increment = shape_mat_[j][i]*mu_meas_[i]*sin(theta)*dtheta_*dphi_;
      y_full_[j] += increment;

      // Front half for steering
      if( phi < M_PI/2 && phi > -M_PI/2){
        y_front_half_[j] += increment;
      }

      // Also do bottom half for ground following
      if( i < last_index_ / 2; i++){
        y_bottom_half_[j] += increment;
      }

    }

  }

  if(enable_debug_){
    y_projections_with_odom_msg_.header.stamp = ros::Time::now();
    y_projections_with_odom_msg_.full_projections = y_full_;
    y_projections_with_odom_msg_.front_half_projections = y_front_half_;
    y_projections_with_odom_msg_.bottom_half_projections = y_bottom_half_;
    y_projections_with_odom_msg_.odometry = current_odom_;
    pub_y_projections_with_odom_.publish(y_projections_with_odom_msg_);
  }

  // We are done processing the current pointcloud
  new_pcl_ = false;

}

void NearnessController3D::reconstructWideFieldNearness(){

  recon_wf_mu_vec_.clear();
  vector<float> zeros(last_index_,0.0);
  recon_wf_mu_vec_ = zeros;
  for(int j = 0 ; j < num_wf_harmonics_; j++){
    for(int i = 0; i < last_index_; i++){
      recon_wf_mu_vec_[i] += y_full_[j]*shape_mat_[j][i];
    }
  }

  pcl::PointXYZ recon_mu_p;
  recon_wf_mu_pcl_.clear();

  if(enable_debug_){
    // Turn reconstructed wf back into pointcloud for viewing
    float theta, phi;
    for(int i = 0; i < last_index_; i++){
      theta = viewing_angle_mat_[i][0];
      phi = viewing_angle_mat_[i][1];

      recon_mu_p = {recon_wf_mu_vec_[i]*sin(theta)*cos(phi), recon_wf_mu_vec_[i]*sin(theta)*sin(phi), recon_wf_mu_vec_[i]*cos(theta) };
      recon_wf_mu_pcl_.push_back(recon_mu_p);

    }

    pcl::toROSMsg(recon_wf_mu_pcl_, recon_wf_mu_pcl_msg_);
    recon_wf_mu_pcl_msg_.header.frame_id = frame_id_;
    recon_wf_mu_pcl_msg_.header.stamp = ros::Time::now();
    pub_recon_wf_mu_.publish(recon_wf_mu_pcl_msg_);
  }
}

void NearnessController3D::computeSmallFieldNearness(){

  sf_mu_.clear();
  float diff, theta, phi;
  pcl::PointXYZ sf_mu_p;
  sf_mu_pcl_.clear();

  for(int i = 0; i< last_index_; i++){
    diff = mu_meas_[i] - recon_wf_mu_vec_[i];
    sf_mu_.push_back(diff);

    if(enable_debug_){
      theta = viewing_angle_mat_[i][0];
      phi = viewing_angle_mat_[i][1];
      diff = abs(diff);
      sf_mu_p = {diff*sin(theta)*cos(phi), diff*sin(theta)*sin(phi), diff*cos(theta) };
      sf_mu_pcl_.push_back(sf_mu_p);
    }
  }

  if(enable_debug_){
    pcl::toROSMsg(sf_mu_pcl_, sf_mu_pcl_msg_);
    sf_mu_pcl_msg_.header.frame_id = frame_id_;
    sf_mu_pcl_msg_.header.stamp = ros::Time::now();
    pub_sf_mu_.publish(sf_mu_pcl_msg_);
  }

}

void NearnessController3D::computeControlCommands(){

  control_commands_.linear.x = 0.0;
  control_commands_.linear.y = 0.0;
  control_commands_.linear.z = 0.0;

  control_commands_.angular.x = 0.0;
  control_commands_.angular.y = 0.0;
  control_commands_.angular.z = 0.0;

  // Compute control commands
  //int num_controls = C_mat_.size();
  //u_vec_.clear();
  // for (int i=0; i < num_controls; i++){
  //   u_vec_.push_back(0.0);
  //   for(int j=0; j < num_basis_shapes_; j++){
  //     u_vec_[i] += C_mat_[i][j]*y_projections_[j];
  //   }
  // }
  u_vec_.clear();
  u_vec_ = {0.0, 0.0, 0.0};
  for(int j=0; j < num_basis_shapes_; j++){
    u_vec_[0] += C_dy_[j]*y_full_[j];
    u_vec_[1] += C_dtheta_[j]*y_front_half_[j];
    u_vec_[2] += C_dz_[j]*y_full_[j];
  }

  // u_vec_.push_back(0.0);
  // for(int j = 0; j < num_basis_shapes_; j++){
  //   u_vec_[2] += C_z_[j]*y_projections_half_[j];
  // }

  if(enable_control_){

    u_v_ = k_v_*u_vec_[0];
    //u_w_ = k_w_*(reference_altitude_ - u_vec_[2]);
    u_w_ = k_w_*u_vec_[2];
    u_thetadot_ = k_thetadot_*u_vec_[1];
    //u_thetadot_ = 0.0;

    if(enable_speed_regulation_){
      //u_u_ =  max_forward_speed_*(1 - k_u_v_*abs(u_v_) - k_u_thetadot_*abs(u_thetadot_));
      //u_u_ =  max_forward_speed_*(1 - k_u_v_*abs(u_v_) - k_u_thetadot_*abs(u_thetadot_) - front_mu_ave_);
      u_u_ =  max_forward_speed_*(1 - front_mu_ave_);
    } else {
      u_u_ = forward_speed_;
    }

    if(enable_cmd_scaling_){

    }

    control_commands_.linear.x = u_u_;
    control_commands_.linear.y = u_v_;
    control_commands_.linear.z = u_w_;
    control_commands_.angular.z = u_thetadot_;

  }

  if(enable_altitude_hold_){
    //ROS_INFO("%f", current_pos_.z);
    u_w_ = k_w_*(reference_altitude_ - current_height_agl_);
    control_commands_.linear.z = u_w_;
  }



  if(sim_control_){
    control_commands_.linear.x = joy_cmd_.linear.x;
    control_commands_.linear.y = joy_cmd_.linear.y;

    if(!enable_altitude_hold_){
      control_commands_.linear.z = joy_cmd_.linear.z;
    }

    control_commands_.angular.z = joy_cmd_.angular.z;
  }

  pub_control_commands_.publish(control_commands_);

    if(enable_debug_){

    //ROS_INFO_THROTTLE(1,"U: %f, V: %f, W: %f, YR: %f", u_u_, u_v_, u_w_, u_thetadot_);

    // Forward speed marker
    ros::Time time_now = ros::Time::now();
    u_cmd_marker_.header.stamp = time_now;
    u_cmd_marker_.points[1].x = u_u_;
    cmd_markers_.markers[0] = u_cmd_marker_;
    //pub_u_cmd_marker_.publish(u_cmd_marker_);

    // Lateral speed marker
    v_cmd_marker_.header.stamp = time_now;
    v_cmd_marker_.points[1].y = u_v_;
    cmd_markers_.markers[1] = v_cmd_marker_;
    //pub_v_cmd_marker_.publish(v_cmd_marker_);

    // Forward speed marker
    w_cmd_marker_.header.stamp = time_now;
    w_cmd_marker_.points[1].z = u_w_;
    cmd_markers_.markers[2] = w_cmd_marker_;
    //pub_w_cmd_marker_.publish(w_cmd_marker_);

    // Turn rate marker
    thetadot_cmd_marker_.header.stamp = time_now;
    thetadot_cmd_marker_.points[1].y = u_thetadot_;
    cmd_markers_.markers[3] = thetadot_cmd_marker_;
    pub_cmd_markers_.publish(cmd_markers_);

  }

}

void NearnessController3D::odomCb(const nav_msgs::OdometryConstPtr& odom_msg){
    current_odom_ = *odom_msg;
    current_pos_ = current_odom_.pose.pose.position;
    current_odom_.pose.pose.position.z = current_height_agl_;
    geometry_msgs::Quaternion vehicle_quat_msg = current_odom_.pose.pose.orientation;
    tf::Quaternion vehicle_quat_tf;
    tf::quaternionMsgToTF(vehicle_quat_msg, vehicle_quat_tf);
    tf::Matrix3x3(vehicle_quat_tf).getRPY(current_roll_, current_pitch_, current_heading_);
}

void NearnessController3D::joyconCb(const sensor_msgs::JoyConstPtr& joy_msg)
{

    // Enable / Disable Altitude Hold
    if(joy_msg->buttons[5] == 1){
      enable_altitude_hold_ = true;
      ROS_INFO_THROTTLE(2,"ALT HOLD ENABLED");
    }
    if(joy_msg->buttons[3] == 1){
      enable_altitude_hold_ = false;
      ROS_INFO_THROTTLE(2,"ALT HOLD DISABLED");
    }

    // Enable / Disable Nearness Control
    if(joy_msg->buttons[0] == 1){
      enable_control_ = true;
      ROS_INFO_THROTTLE(2,"NEARNESS CONTROL ENABLED");
    }
    if(joy_msg->buttons[1] == 1){
      enable_control_ = false;
      ROS_INFO_THROTTLE(2,"NEARNESS CONTROL DISABLED");
    }

    if(joy_msg->buttons[4] == 1){
      sim_control_ = true;
      joy_cmd_.linear.x = joy_msg->axes[4]*max_forward_speed_;
      joy_cmd_.linear.y = joy_msg->axes[3]*max_lateral_speed_;
      joy_cmd_.linear.z = joy_msg->axes[1]*max_vertical_speed_;
      joy_cmd_.angular.z = joy_msg->axes[0]*max_yaw_rate_;
    } else {
      sim_control_ =false;
      joy_cmd_.linear.x = 0.0;
      joy_cmd_.linear.y = 0.0;
      joy_cmd_.linear.z = 0.0;
      joy_cmd_.angular.z = 0.0;
    }

}

void NearnessController3D::generateProjectionShapes(){

    // This makes rviz plotting look nice
    // Positive is red, negative is blue with this config
    float intensity_val = 0.0;
    float max_intensity = -1.0;
    float min_intensity = .6;
    float theta, phi, d, d_abs;

    // Cycle through every point in the spherical coordinate system
    // and generate the Laplace Spherical harmonic shapes. Cycle
    // from bottom to top ring, and pi to -pi along a ring.
    for(int i = 1; i < num_rings_-1; i++){
      // Get current theta value
      theta = theta_view_vec_[i];

      for(int j = 0; j < num_ring_points_; j++){
        // Get current phi value
        phi = phi_view_vec_[j];

        // Y00
        d = sqrt(1/(4*M_PI));
        d_abs = abs(d);
        Y00_vec_.push_back(d);
        pcl::PointXYZ Y00_pcl (d_abs*sin(theta)*cos(phi), d_abs*sin(theta)*sin(phi), d_abs*cos(theta) );
        pcl::PointXYZI Y00_pcli;
        Y00_pcli.x = Y00_pcl.x; Y00_pcli.y = Y00_pcl.y; Y00_pcli.z = Y00_pcl.z;
        if(d >= 0){
          intensity_val = max_intensity;
        } else {
          intensity_val = min_intensity;
        }
        Y00_pcli.intensity = intensity_val;
        Y00_.push_back(Y00_pcli);

        // Y0p1
        d = sqrt(3.0/(4.0*M_PI))*cos(theta);
        d_abs = abs(d);
        Y0p1_vec_.push_back(d);
        pcl::PointXYZ Y0p1_pcl (d_abs*sin(theta)*cos(phi), d_abs*sin(theta)*sin(phi), d_abs*cos(theta) );
        pcl::PointXYZI Y0p1_pcli;
        Y0p1_pcli.x = Y0p1_pcl.x; Y0p1_pcli.y = Y0p1_pcl.y; Y0p1_pcli.z = Y0p1_pcl.z;
        if(d >= 0){
          intensity_val = max_intensity;
        } else {
          intensity_val = min_intensity;
        }
        Y0p1_pcli.intensity = intensity_val;
        Y0p1_.push_back(Y0p1_pcli);

        // Yp1p1
        d = sqrt(3.0/(4.0*M_PI))*cos(phi)*sin(theta);
        d_abs = abs(d);
        Yp1p1_vec_.push_back(d);
        pcl::PointXYZ Yp1p1_pcl (d_abs*sin(theta)*cos(phi), d_abs*sin(theta)*sin(phi), d_abs*cos(theta) );
        pcl::PointXYZI Yp1p1_pcli;
        Yp1p1_pcli.x = Yp1p1_pcl.x; Yp1p1_pcli.y = Yp1p1_pcl.y; Yp1p1_pcli.z = Yp1p1_pcl.z;
        if(d >= 0){
          intensity_val = max_intensity;
        } else {
          intensity_val = min_intensity;
        }
        Yp1p1_pcli.intensity = intensity_val;
        Yp1p1_.push_back(Yp1p1_pcli);

        // Yn1p1
        d = sqrt(3.0/(4.0*M_PI))*sin(phi)*sin(theta);
        d_abs = abs(d);
        Yn1p1_vec_.push_back(d);
        pcl::PointXYZ Yn1p1_pcl (d_abs*sin(theta)*cos(phi), d_abs*sin(theta)*sin(phi), d_abs*cos(theta) );
        pcl::PointXYZI Yn1p1_pcli;
        Yn1p1_pcli.x = Yn1p1_pcl.x; Yn1p1_pcli.y = Yn1p1_pcl.y; Yn1p1_pcli.z = Yn1p1_pcl.z;
        if(sgn(d) > 0){
          intensity_val = max_intensity;
        } else {
          intensity_val = min_intensity;
        }
        Yn1p1_pcli.intensity = intensity_val;
        Yn1p1_.push_back(Yn1p1_pcli);

        // Y0p2
        d = sqrt(5.0/(16.0*M_PI))*(3.0*pow(cos(theta),2)-1);
        d_abs = abs(d);
        Y0p2_vec_.push_back(d);
        pcl::PointXYZ Y0p2_pcl (d_abs*sin(theta)*cos(phi), d_abs*sin(theta)*sin(phi), d_abs*cos(theta) );
        pcl::PointXYZI Y0p2_pcli;
        Y0p2_pcli.x = Y0p2_pcl.x; Y0p2_pcli.y = Y0p2_pcl.y; Y0p2_pcli.z = Y0p2_pcl.z;
        if(d >= 0){
          intensity_val = max_intensity;
        } else {
          intensity_val = min_intensity;
        }
        Y0p2_pcli.intensity = intensity_val;
        Y0p2_.push_back(Y0p2_pcli);

        // Yp1p2
        d = sqrt(15.0/(4.0*M_PI))*cos(phi)*sin(theta)*cos(theta);
        d_abs = abs(d);
        Yp1p2_vec_.push_back(d);
        pcl::PointXYZ Yp1p2_pcl (d_abs*sin(theta)*cos(phi), d_abs*sin(theta)*sin(phi), d_abs*cos(theta) );
        pcl::PointXYZI Yp1p2_pcli;
        Yp1p2_pcli.x = Yp1p2_pcl.x; Yp1p2_pcli.y = Yp1p2_pcl.y; Yp1p2_pcli.z = Yp1p2_pcl.z;
        if(d >= 0){
          intensity_val = max_intensity;
        } else {
          intensity_val = min_intensity;
        }
        Yp1p2_pcli.intensity = intensity_val;
        Yp1p2_.push_back(Yp1p2_pcli);

        //  Yn1p2
        d = sqrt(15.0/(4.0*M_PI))*sin(phi)*sin(theta)*cos(theta);
        d_abs = abs(d);
        Yn1p2_vec_.push_back(d);
        pcl::PointXYZ Yn1p2_pcl (d_abs*sin(theta)*cos(phi), d_abs*sin(theta)*sin(phi), d_abs*cos(theta) );
        pcl::PointXYZI Yn1p2_pcli;
        Yn1p2_pcli.x = Yn1p2_pcl.x; Yn1p2_pcli.y = Yn1p2_pcl.y; Yn1p2_pcli.z = Yn1p2_pcl.z;
        if(d >= 0){
          intensity_val = max_intensity;
        } else {
          intensity_val = min_intensity;
        }
        Yn1p2_pcli.intensity = intensity_val;
        Yn1p2_.push_back(Yn1p2_pcli);

        // Yp2p2
        d = sqrt(15.0/(16.0*M_PI))*(pow(cos(phi),2)-pow(sin(phi),2))*pow(sin(theta),2);
        d_abs = abs(d);
        Yp2p2_vec_.push_back(d);
        pcl::PointXYZ Yp2p2_pcl (d_abs*sin(theta)*cos(phi), d_abs*sin(theta)*sin(phi), d_abs*cos(theta) );
        pcl::PointXYZI Yp2p2_pcli;
        Yp2p2_pcli.x = Yp2p2_pcl.x; Yp2p2_pcli.y = Yp2p2_pcl.y; Yp2p2_pcli.z = Yp2p2_pcl.z;
        if(d >= 0){
          intensity_val = max_intensity;
        } else {
          intensity_val = min_intensity;
        }
        Yp2p2_pcli.intensity = intensity_val;
        Yp2p2_.push_back(Yp2p2_pcli);

        // Yn2p2
        d = sqrt(15.0/(4.0*M_PI))*sin(phi)*cos(phi)*pow(sin(theta),2);
        d_abs = abs(d);
        Yn2p2_vec_.push_back(d);
        pcl::PointXYZ Yn2p2_pcl (d_abs*sin(theta)*cos(phi), d_abs*sin(theta)*sin(phi), d_abs*cos(theta) );
        pcl::PointXYZI Yn2p2_pcli;
        Yn2p2_pcli.x = Yn2p2_pcl.x; Yn2p2_pcli.y = Yn2p2_pcl.y; Yn2p2_pcli.z = Yn2p2_pcl.z;
        if(d >= 0){
          intensity_val = max_intensity;
        } else {
          intensity_val = min_intensity;
        }
        Yn2p2_pcli.intensity = intensity_val;
        Yn2p2_.push_back(Yn2p2_pcli);

      }

    }

    // Create array of projection shape arrays
    // Indexing is [shape][i*ring_num + j]
    // for i rings and j points per ring
    shape_mat_.push_back(Y00_vec_);
    shape_mat_.push_back(Y0p1_vec_);
    shape_mat_.push_back(Yp1p1_vec_);
    shape_mat_.push_back(Yn1p1_vec_);
    shape_mat_.push_back(Y0p2_vec_);
    shape_mat_.push_back(Yp1p2_vec_);
    shape_mat_.push_back(Yn1p2_vec_);
    shape_mat_.push_back(Yp2p2_vec_);
    shape_mat_.push_back(Yn2p2_vec_);

    // Generate state projection shapes
    // y-state projection shape
    float d_y ,d_theta, d_abs_y, d_abs_theta;
    float d_z, d_abs_z;
    for(int i = 0; i < last_index_; i++){
      d_y = 0.0;
      d_theta = 0.0;
      d_z = 0.0;
      for (int k = 0; k < num_basis_shapes_; k++){
        d_y += C_dy_[k]*shape_mat_[k][i];
        d_theta += C_dtheta_[k]*shape_mat_[k][i];
        if(k < last_index_ / 2){
          d_z += C_dz_[k]*shape_mat_[k][i];
        }
      }
      y_projection_shape_vec_.push_back(d_y);
      theta_projection_shape_vec_.push_back(d_theta);
      z_projection_shape_vec_.push_back(d_z);

      theta = viewing_angle_mat_[i][0];
      phi = viewing_angle_mat_[i][1];

      d_abs_y = abs(d_y);
      pcl::PointXYZ y_pcl (d_abs_y*sin(theta)*cos(phi), d_abs_y*sin(theta)*sin(phi), d_abs_y*cos(theta) );
      pcl::PointXYZI y_pcli;
      y_pcli.x = y_pcl.x; y_pcli.y = y_pcl.y; y_pcli.z = y_pcl.z;
      if(d_y >= 0){
        intensity_val = max_intensity;
      } else {
        intensity_val = min_intensity;
      }
      y_pcli.intensity = intensity_val;
      y_projection_shape_.push_back(y_pcli);

      d_abs_theta = abs(d_theta);
      pcl::PointXYZ theta_pcl (d_abs_theta*sin(theta)*cos(phi), d_abs_theta*sin(theta)*sin(phi), d_abs_theta*cos(theta) );
      pcl::PointXYZI theta_pcli;
      theta_pcli.x = theta_pcl.x; theta_pcli.y = theta_pcl.y; theta_pcli.z = theta_pcl.z;
      if(d_theta >= 0){
        intensity_val = max_intensity;
      } else {
        intensity_val = min_intensity;
      }
      theta_pcli.intensity = intensity_val;
      theta_projection_shape_.push_back(theta_pcli);

      d_abs_z = abs(d_z);
      pcl::PointXYZ z_pcl (d_abs_z*sin(theta)*cos(phi), d_abs_z*sin(theta)*sin(phi), d_abs_z*cos(theta) );
      pcl::PointXYZI z_pcli;
      z_pcli.x = z_pcl.x; z_pcli.y = z_pcl.y; z_pcli.z = z_pcl.z;
      if(d_z >= 0){
        intensity_val = max_intensity;
      } else {
        intensity_val = min_intensity;
      }
      z_pcli.intensity = intensity_val;
      z_projection_shape_.push_back(z_pcli);

    }

    // This is strictly for making rviz look nice
    // Makes positive values red and negative values blue
    pcl::PointXYZI Yp_pcli;
    Yp_pcli.x = 0.0; Yp_pcli.y = 0.0; Yp_pcli.z = 0.0;
    Yp_pcli.intensity = 1.0;
    Y00_.push_back(Yp_pcli);
    Y0p1_.push_back(Yp_pcli);
    Yp1p1_.push_back(Yp_pcli);
    Yn1p1_.push_back(Yp_pcli);
    Y0p2_.push_back(Yp_pcli);
    Yp1p2_.push_back(Yp_pcli);
    Yn1p2_.push_back(Yp_pcli);
    Yp2p2_.push_back(Yp_pcli);
    Yn2p2_.push_back(Yp_pcli);
    y_projection_shape_.push_back(Yp_pcli);
    theta_projection_shape_.push_back(Yp_pcli);
    z_projection_shape_.push_back(Yp_pcli);
    pcl::PointXYZI Yn_pcli;
    Yn_pcli.x = 0.0; Yn_pcli.y = 0.0; Yn_pcli.z = 0.0;
    Yn_pcli.intensity = -1.0;
    Y00_.push_back(Yn_pcli);
    Y0p1_.push_back(Yn_pcli);
    Yp1p1_.push_back(Yn_pcli);
    Yn1p1_.push_back(Yn_pcli);
    Y0p2_.push_back(Yn_pcli);
    Yp1p2_.push_back(Yn_pcli);
    Yn1p2_.push_back(Yn_pcli);
    Yp2p2_.push_back(Yn_pcli);
    Yn2p2_.push_back(Yn_pcli);
    y_projection_shape_.push_back(Yn_pcli);
    theta_projection_shape_.push_back(Yn_pcli);
    z_projection_shape_.push_back(Yn_pcli);

    pcl::toROSMsg(Y00_, Y00_msg_);
    Y00_msg_.header.frame_id = frame_id_;

    pcl::toROSMsg(Y0p1_, Y0p1_msg_);
    Y0p1_msg_.header.frame_id = frame_id_;

    pcl::toROSMsg(Yp1p1_, Yp1p1_msg_);
    Yp1p1_msg_.header.frame_id = frame_id_;

    pcl::toROSMsg(Yn1p1_, Yn1p1_msg_);
    Yn1p1_msg_.header.frame_id = frame_id_;

    pcl::toROSMsg(Y0p2_, Y0p2_msg_);
    Y0p2_msg_.header.frame_id = frame_id_;

    pcl::toROSMsg(Yp1p2_, Yp1p2_msg_);
    Yp1p2_msg_.header.frame_id = frame_id_;

    pcl::toROSMsg(Yn1p2_, Yn1p2_msg_);
    Yn1p2_msg_.header.frame_id = frame_id_;

    pcl::toROSMsg(Yp2p2_, Yp2p2_msg_);
    Yp2p2_msg_.header.frame_id = frame_id_;

    pcl::toROSMsg(Yn2p2_, Yn2p2_msg_);
    Yn2p2_msg_.header.frame_id = frame_id_;

    pcl::toROSMsg(y_projection_shape_, y_projection_shape_msg_);
    y_projection_shape_msg_.header.frame_id = frame_id_;

    pcl::toROSMsg(theta_projection_shape_, theta_projection_shape_msg_);
    theta_projection_shape_msg_.header.frame_id = frame_id_;

    pcl::toROSMsg(z_projection_shape_, z_projection_shape_msg_);
    z_projection_shape_msg_.header.frame_id = frame_id_;

}

void NearnessController3D::publishProjectionShapes(){

    ros::Time time_now = ros::Time::now();

    Y00_msg_.header.stamp = time_now;
    pub_Y00_.publish(Y00_msg_);

    Y0p1_msg_.header.stamp = time_now;
    pub_Y0p1_.publish(Y0p1_msg_);

    Yp1p1_msg_.header.stamp = time_now;
    pub_Yp1p1_.publish(Yp1p1_msg_);

    Yn1p1_msg_.header.stamp = time_now;
    pub_Yn1p1_.publish(Yn1p1_msg_);

    Y0p2_msg_.header.stamp = time_now;
    pub_Y0p2_.publish(Y0p2_msg_);

    Yp1p2_msg_.header.stamp = time_now;
    pub_Yp1p2_.publish(Yp1p2_msg_);

    Yn1p2_msg_.header.stamp = time_now;
    pub_Yn1p2_.publish(Yn1p2_msg_);

    Yp2p2_msg_.header.stamp = time_now;
    pub_Yp2p2_.publish(Yp2p2_msg_);

    Yn2p2_msg_.header.stamp = time_now;
    pub_Yn2p2_.publish(Yn2p2_msg_);

    y_projection_shape_msg_.header.stamp = time_now;
    pub_y_projection_shape_.publish(y_projection_shape_msg_);

    theta_projection_shape_msg_.header.stamp = time_now;
    pub_theta_projection_shape_.publish(theta_projection_shape_msg_);

    z_projection_shape_msg_.header.stamp = time_now;
    pub_z_projection_shape_.publish(z_projection_shape_msg_);

}

float NearnessController3D::sgn(double v) {
    return (v < 0.0) ? -1.0 : ((v > 0.0) ? 1.0 : 0.0);
}

float NearnessController3D::wrapAngle(float angle){
    if (angle > M_PI){
        angle -= 2*M_PI;
    } else if( angle < -M_PI){
        angle += 2*M_PI;
    }
    return angle;
}

float NearnessController3D::sat(float num, float min_val, float max_val){
    if (num >= max_val){
        return max_val;
    } else if( num <= min_val){
         return min_val;
    } else {
      return num;
    }
}

int NearnessController3D::fact(int n)
{
    if(n > 1)
        return n * fact(n - 1);
    else
        return 1;
}

 // end of class
} // End of namespace nearness
