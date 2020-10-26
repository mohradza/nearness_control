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
    pub_y_projections_ = nh_.advertise<std_msgs::Float32MultiArray>("y_projections",1);
    pub_recon_wf_mu_ = nh_.advertise<sensor_msgs::PointCloud2>("reconstructed_wf_nearness",1);
    pub_sf_mu_ = nh_.advertise<sensor_msgs::PointCloud2>("sf_nearness",1);
    pub_control_commands_ = nh_.advertise<geometry_msgs::TwistStamped>("control_commands",1);

    // Import parameters
    pnh_.param("enable_debug", enable_debug_, false);
    pnh_.param("enable_altitude_hold", enable_altitude_hold_, false);

    pnh_.param("num_rings", num_rings_, 64);
    pnh_.param("num_ring_points", num_ring_points_, 360);
    pnh_.param("num_basis_shapes", num_basis_shapes_, 9);
    pnh_.param("num_wf_harmonics", num_wf_harmonics_, 9);

    pnh_.param("altitude_hold_gain", k_alt_, -0.1);
    pnh_.param("lateral_speed_gain", k_v_, -0.1);
    pnh_.param("turn_rate_gain", k_thetadot_, -0.1);
    pnh_.param("vertical_speed_gain", k_w_, -0.1);
    pnh_.param("forward_speed", u_u_, .5);

    frame_id_ = "OHRAD_X3";

    control_commands_.header.frame_id = frame_id_;

    // We want to exclude the top and bottom rings
    num_excluded_rings_ = 2;
    last_index_ = (num_rings_- num_excluded_rings_)*num_ring_points_;

    new_pcl_ = false;

    // Set up the Cdagger matrix
    // Will be referenced as C in code
    // Might want to implement this with Eigen
    vector<float> zeros_vec(num_basis_shapes_, 0.0);
    C_dy_ = zeros_vec;
    C_mat_.push_back(C_dy_);
    C_dtheta_ = zeros_vec;
    C_mat_.push_back(C_dtheta_);
    C_dz_ = zeros_vec;
    C_mat_.push_back(C_dz_);

    // Prepare the Laplace spherical harmonic basis set
    generateViewingAngleVectors();
    generateProjectionShapes();


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

  pcl::PointXYZ p, mu_p;
  float dist, mu_val;
  for(int i = 1; i < num_rings_-1; i++){
      for(int j = 0; j < num_ring_points_; j++){
          p = cloud_in->points[i*num_ring_points_ + j];
          cloud_out_.push_back(p);
          dist = sqrt(pow(p.x,2) + pow(p.y,2) + pow(p.z,2));
          mu_val = 1/dist;
          mu_meas_.push_back(mu_val);

          // Convert back to cartesian for viewing
          if(enable_debug_){
              mu_p = {mu_val*sin(theta_view_vec_[i])*cos(phi_view_vec_[j]), mu_val*sin(theta_view_vec_[i])*sin(phi_view_vec_[j]), mu_val*cos(theta_view_vec_[i]) };
              mu_cloud_out_.push_back(mu_p);
          }
      }
  }

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
  y_projections_.clear();
  for(int j = 0; j < num_basis_shapes_; j++){
    y_projections_.push_back(0.0);
    for (int i = 0; i < last_index_-400; i++){
      // float num1 = shape_mat_[j][i];
      //float num2 = mu_sphere_[i];
      y_projections_[j] += shape_mat_[j][i]*mu_meas_[i]*sin(viewing_angle_mat_[i][0])*dtheta_*dphi_;
    }
  }

  if(enable_debug_){
    y_projections_msg_.data = y_projections_;
    pub_y_projections_.publish(y_projections_msg_);
  }

  new_pcl_ = false;

}

void NearnessController3D::reconstructWideFieldNearness(){

  recon_wf_mu_vec_.clear();
  vector<float> zeros(last_index_,0.0);
  recon_wf_mu_vec_ = zeros;
  for(int j = 0 ; j < num_wf_harmonics_; j++){
    for(int i = 0; i < last_index_; i++){
      recon_wf_mu_vec_[i] += y_projections_[j]*shape_mat_[j][i];
    }
  }

  pcl::PointXYZ recon_mu_p;
  recon_wf_mu_pcl_.clear();

  if(enable_debug_){
    // Turn reconstructed wf back into pointcloud for viewing
    float theta, phi;
    for(int i = 0; i< last_index_; i++){
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

  int num_controls = C_mat_.size();
  u_vec_.clear();
  for (int i=0; i < num_controls; i++){
    u_vec_.push_back(0.0);
    for(int j=0; j < num_basis_shapes_; j++){
      u_vec_[i] += C_mat_[i][j]*y_projections_[j];
    }
  }

  u_v_ = k_v_*u_vec_[0];
  u_thetadot_ = k_thetadot_*u_vec_[1];
  u_w_ = k_w_*u_vec_[2];

  if(enable_altitude_hold_){
    u_w_ = k_alt_*(reference_altitude_ - current_pos_.z);
  }

  control_commands_.header.stamp = ros::Time::now();
  control_commands_.twist.linear.x = u_u_;
  control_commands_.twist.linear.y = u_v_;
  control_commands_.twist.linear.z = u_w_;

  control_commands_.twist.angular.x = 0.0;
  control_commands_.twist.angular.y = 0.0;
  control_commands_.twist.angular.z = u_thetadot_;

  pub_control_commands_.publish(control_commands_);

}

void NearnessController3D::odomCb(const nav_msgs::OdometryConstPtr& odom_msg){
    current_pos_ = odom_msg->pose.pose.position;
    geometry_msgs::Quaternion vehicle_quat_msg = odom_msg->pose.pose.orientation;
    tf::Quaternion vehicle_quat_tf;
    tf::quaternionMsgToTF(vehicle_quat_msg, vehicle_quat_tf);
    tf::Matrix3x3(vehicle_quat_tf).getRPY(current_roll_, current_pitch_, current_heading_);

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
