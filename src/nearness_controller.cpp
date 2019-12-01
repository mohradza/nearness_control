#include <nearness_control/nearness_controller.h>

namespace nearness{
NearnessController::NearnessController(const ros::NodeHandle &node_handle,
                                       const ros::NodeHandle &private_node_handle)
      :nh_(node_handle),
       pnh_(private_node_handle) {
      this->init();
  }

void NearnessController::init() {
    // Set up dynamic reconfigure
    reconfigure_server_.reset(new ReconfigureServer(config_mutex_, pnh_));
    ReconfigureServer::CallbackType f = boost::bind(&NearnessController::configCb, this, _1, _2);
    reconfigure_server_->setCallback(f);

    debug_ = true;
    is_ground_vehicle_ = false;
    flag_estop_ = true;
    control_command_.header.frame_id = "/base_stabilized";

    // Set up subscribers and callbacks
    sub_horiz_laserscan_ = nh_.subscribe("horiz_scan", 1, &NearnessController::horizLaserscanCb, this);
    sub_vert_laserscan_ = nh_.subscribe("vert_scan", 1, &NearnessController::vertLaserscanCb, this);
    sub_odom_ = nh_.subscribe("odometry", 1, &NearnessController::odomCb, this);
    sub_bluetooth_joy_ = nh_.subscribe("joy", 1, &NearnessController::joyconCb, this);

    //sub_imu_ = nh_.subscribe("imu_raw", 1, &NearnessController::imuCb, this);
    sub_sonar_height_ = nh_.subscribe("/sonar_height", 1, &NearnessController::sonarHeightCb, this);

    sub_next_waypoint_ = nh_.subscribe("next_waypoint", 1, &NearnessController::nextWaypointCb, this);

    // Set up publishers
    pub_h_scan_reformat_ = nh_.advertise<std_msgs::Float32MultiArray>("horiz_depth_reformat", 10);
    pub_h_scan_nearness_ = nh_.advertise<std_msgs::Float32MultiArray>("horiz_nearness", 10);
    pub_h_sf_nearness_ = nh_.advertise<std_msgs::Float32MultiArray>("horiz_sf_nearness", 10);
    pub_h_recon_wf_nearness_ = nh_.advertise<std_msgs::Float32MultiArray>("horiz_recon_wf_nearness", 10);
    pub_h_fourier_coefficients_ = nh_.advertise<nearness_control::FourierCoefsMsg>("horiz_fourier_coefficients", 10);
    pub_v_scan_reformat_ = nh_.advertise<std_msgs::Float32MultiArray>("vert_depth_reformat", 10);
    pub_v_scan_nearness_ = nh_.advertise<std_msgs::Float32MultiArray>("vert_nearness", 10);
    pub_v_sf_nearness_ = nh_.advertise<std_msgs::Float32MultiArray>("vert_sf_nearness", 10);
    pub_v_recon_wf_nearness_ = nh_.advertise<std_msgs::Float32MultiArray>("vert_recon_wf_nearness", 10);
    pub_v_fourier_coefficients_ = nh_.advertise<nearness_control::FourierCoefsMsg>("vert_fourier_coefficients", 10);
    pub_control_commands_ = nh_.advertise<geometry_msgs::TwistStamped>("control_commands", 10);
    pub_sim_control_commands_ = nh_.advertise<geometry_msgs::Twist>("sim_control_commands", 10);
    pub_h_sf_yawrate_command_ = nh_.advertise<std_msgs::Float32>("sf_yawrate_command", 10);
    pub_vehicle_status_ = nh_.advertise<std_msgs::Int32>("vehicle_status", 10);
    pub_estop_engage_ = nh_.advertise<std_msgs::Bool>("estop_cmd", 10);


    // Import parameters
    // Sensor
    h_num_fourier_terms_ = 5;
    v_num_fourier_terms_ = 5;
    enable_gain_scaling_ = true;
    enable_sf_control_ = false;
    enable_attractor_control_ = false;
    have_attractor_ = false;
    enable_wf_control_ = true;

    nh_.param("/nearness_control_node/total_horiz_scan_points", total_h_scan_points_, 1440);
    nh_.param("/nearness_control_node/horiz_scan_limit", h_scan_limit_, M_PI);
    nh_.param("/nearness_control_node/num_horiz_scan_points", num_h_scan_points_, 720);
    //nh_.param("/nearness_control_node/scan_start_location", scan_start_loc_, "back");
    nh_.param("/nearness_control_node/horiz_sensor_min_distance", h_sensor_min_dist_, .1);
    nh_.param("/nearness_control_node/horiz_sensor_max_distance", h_sensor_max_dist_, 25.0);
    nh_.param("/nearness_control_node/horiz_scan_start_index", h_scan_start_index_, 0);
    ROS_INFO("%d", h_scan_start_index_);
    nh_.param("/nearness_control_node/horiz_sensor_min_noise", h_sensor_min_noise_ , .1);
    nh_.param("/nearness_control_node/reverse_horiz_scan", reverse_h_scan_, true);

    nh_.param("/nearness_control_node/total_vert_scan_points", total_v_scan_points_, 1440);
    nh_.param("/nearness_control_node/num_vert_scan_points", num_v_scan_points_, 720);
    nh_.param("/nearness_control_node/vert_scan_limit", v_scan_limit_, M_PI);
    //nh_.param("//nearness_control_node/scan_start_location", scan_start_loc_, "back");
    nh_.param("/nearness_control_node/vert_sensor_min_distance", v_sensor_min_dist_, .1);
    nh_.param("/nearness_control_node/vert_sensor_max_distance", v_sensor_max_dist_, 25.0);
    nh_.param("/nearness_control_node/vert_scan_start_index", v_scan_start_index_, 0);
    nh_.param("/nearness_control_node/vert_sensor_min_noise", v_sensor_min_noise_ , .1);
    nh_.param("/nearness_control_node/reverse_vert_scan", reverse_v_scan_, false);

    // Safety
    nh_.param("/nearness_control_node/enable_safety_boundary", enable_safety_boundary_, false);
    nh_.param("/nearness_control_node/enable_safety_box", enable_safety_box_, false);
    nh_.param("/nearness_control_node/enable_safety_radius", enable_safety_radius_, false);
    nh_.param("/nearness_control_node/front_safety_distance", f_dist_, .5);
    nh_.param("/nearness_control_node/side_safety_distance", s_dist_, .5);
    nh_.param("/nearness_control_node/safety_radius", safety_radius_, .5);

    // Wide Field controller gains
    nh_.param("/nearness_control_node/forward_speed_k_hb_1", u_k_hb_1_, 0.0);
    nh_.param("/nearness_control_node/forward_speed_k_hb_2", u_k_hb_2_, 3.5);
    nh_.param("/nearness_control_node/forward_speed_k_vb_1", u_k_vb_1_, 0.0);
    nh_.param("/nearness_control_node/forward_speed_k_vb_2", u_k_vb_2_, 3.5);
    nh_.param("/nearness_control_node/forward_speed_min", u_min_, .1);
    nh_.param("/nearness_control_node/forward_speed_max", u_max_, 5.0);
    nh_.param("/nearness_control_node/yaw_rate_k_hb_1", r_k_hb_1_, 2.0);
    nh_.param("/nearness_control_node/yaw_rate_k_hb_2", r_k_hb_2_, 2.0);
    nh_.param("/nearness_control_node/yaw_rate_k_vb_1", r_k_vb_1_, 2.0);
    nh_.param("/nearness_control_node/yaw_rate_k_vb_2", r_k_vb_2_, 2.0);
    nh_.param("/nearness_control_node/yaw_rate_k_att_0", r_k_att_0_, 1.0);
    nh_.param("/nearness_control_node/yaw_rate_k_att_d_", r_k_att_d_, 0.1);
    nh_.param("/nearness_control_node/yaw_rate_max", r_max_, 2.0);
    nh_.param("/nearness_control_node/h_sf_k_0", h_sf_k_0_, 2.0);
    nh_.param("/nearness_control_node/h_sf_k_d", h_sf_k_d_, 2.0);
    nh_.param("/nearness_control_node/h_sf_k_psi", h_sf_k_psi_, 2.0);
    nh_.param("/nearness_control_node/h_sf_k_thresh", h_sf_k_thresh_, 2.0);
    nh_.param("/nearness_control_node/v_sf_k_0", v_sf_k_0_, 2.0);
    nh_.param("/nearness_control_node/v_sf_k_d", v_sf_k_d_, 2.0);
    nh_.param("/nearness_control_node/v_sf_k_psi", v_sf_k_psi_, 2.0);
    nh_.param("/nearness_control_node/v_sf_k_thresh", v_sf_k_thresh_, 2.0);

    nh_.param("/nearness_control_node/vert_speed_k_vb_1", w_k_1_, 2.0);
    nh_.param("/nearness_control_node/vert_speed_k_vb_2", w_k_2_, 2.0);
    nh_.param("/nearness_control_node/vert_speed_max", w_max_, 2.0);


    if(enable_gain_scaling_){
      r_k_hb_2_ = 1.0*u_max_;
      v_k_hb_1_ = 1.0*u_max_;
    }

    // Generate the gamma vector
    for(int i=0; i<num_h_scan_points_; i++){
        h_gamma_vector_.push_back((float(i)/float(num_h_scan_points_))*(2*h_scan_limit_) - h_scan_limit_);
    }
    h_dg_ = (2.0*h_scan_limit_)/num_h_scan_points_;

    // Generate the gamma vector
    for(int i=0; i<num_v_scan_points_; i++){
        v_gamma_vector_.push_back((float(i)/float(num_v_scan_points_))*(2*v_scan_limit_) - v_scan_limit_);
    }
    v_dg_ = (2.0*v_scan_limit_)/num_v_scan_points_;

    // Create safety boundary
    if(enable_safety_boundary_){
      ROS_INFO("Enabling safety boundary.");
        // Generate safety box or radius around vehicle
        if(enable_safety_box_ && !enable_safety_radius_){
            generateSafetyBox();
        } else if(enable_safety_radius_ && !enable_safety_box_){
            ROS_INFO("Generating safety radius.");
            for(int i=0; i<num_h_scan_points_; i++){
                safety_boundary_.push_back(safety_radius_);
            }
            left_corner_index_ = num_h_scan_points_/4;

        } else if(enable_safety_box_ && enable_safety_radius_){
            ROS_INFO("Cannot have safety box and safety radius. Disabling safety boundary.");
            enable_safety_boundary_ = false;
        }
    }
} // End of init

void NearnessController::configCb(Config &config, uint32_t level)
{
    config_ = config;

    // Controller gains
    u_k_hb_1_ = config_.forward_speed_k_hb_1;
    u_k_hb_2_ = config_.forward_speed_k_hb_2;
    u_k_vb_1_ = config_.forward_speed_k_vb_1;
    u_k_vb_2_ = config_.forward_speed_k_vb_2;
    u_min_ = config_.forward_speed_min;
    u_max_ = config_.forward_speed_max;

    r_k_hb_1_ = config_.yaw_rate_k_hb_1;
    r_k_hb_2_ = config_.yaw_rate_k_hb_2;
    r_max_ = config_.yaw_rate_max;

    r_k_att_0_ = config.yaw_rate_k_att_0;
    r_k_att_d_ = config.yaw_rate_k_att_d;

    h_sf_k_thresh_ = config_.h_sf_k_thresh;
    h_sf_k_0_ = config_.h_sf_k_0;
    h_sf_k_psi_ = config_.h_sf_k_psi;
    h_sf_k_d_ = config_.h_sf_k_d;

    v_sf_k_thresh_ = config_.v_sf_k_thresh;
    v_sf_k_0_ = config_.v_sf_k_0;
    v_sf_k_psi_ = config_.v_sf_k_psi;
    v_sf_k_d_ = config_.v_sf_k_d;

    v_k_hb_1_ = config_.lateral_speed_k_hb_1;
    v_max_ = config_.lateral_speed_max;

    w_k_1_ = config_.vert_speed_k_vb_1;
    w_k_2_ = config_.vert_speed_k_vb_2;
    w_max_ = config_.vert_speed_max;

    if(enable_gain_scaling_){
      r_k_hb_2_ = 1.0*u_max_;
      v_k_hb_1_ = 1.0*u_max_;
    }

    //ROS_INFO("%f, %f", w_max_, u_min_);
}

void NearnessController::horizLaserscanCb(const sensor_msgs::LaserScanPtr h_laserscan_msg){

    // Convert incoming scan to cv matrix and reformat
    convertHLaserscan2CVMat(h_laserscan_msg);
/*
    // Compute the Fourier harmonics of the signal
    computeHorizFourierCoeffs();

    // Feed back fourier coefficients for forward speed regulation
    computeForwardSpeedCommand();

    computeWFYawRateCommand();

    if(!is_ground_vehicle_){
        computeLateralSpeedCommand();
    }

    if(enable_sf_control_){
        computeSFYawRateCommand();
    }

    if(enable_attractor_control_){
        computeAttractorCommand();
    }

    publishControlCommandMsg();
*/
}

void NearnessController::vertLaserscanCb(const sensor_msgs::LaserScanPtr v_laserscan_msg){
  // Convert incoming scan to cv matrix and reformat
  convertVLaserscan2CVMat(v_laserscan_msg);

  computeVertFourierCoeffs();

  computeWFVerticalSpeedCommand();

  computeSFVerticalSpeedCommand();

}

void NearnessController::convertHLaserscan2CVMat(const sensor_msgs::LaserScanPtr h_laserscan_msg){
    std::vector<float> h_depth_vector = h_laserscan_msg->ranges;

    // handle infs due to sensor max distance
    for(int i = 0; i<total_h_scan_points_; i++){
        if(isinf(h_depth_vector[i])){
            h_depth_vector[i] = h_sensor_max_dist_;
        }
    }

    // Reverse the direction
    // Only reverse if the scan point indices are positive ccw
    if(reverse_h_scan_){
        std::reverse(h_depth_vector.begin(), h_depth_vector.end());
    }

    // Reformat the depth scan depending on the orientation of the scanner
    // scan_start_loc describes the location of the first scan index
    std::vector<float> h_depth_vector_reformat;
    h_scan_start_loc_.data = "back";
    if (h_scan_start_loc_.data == "forward"){
        h_depth_vector_reformat = h_depth_vector;
    } else if (h_scan_start_loc_.data == "right"){
        for (int i = 3*total_h_scan_points_/4; i < total_h_scan_points_; i++) {
            h_depth_vector_reformat.push_back(h_depth_vector[i]);
        }
        for (int i = 0; i < 3*total_h_scan_points_/4; i++){
            h_depth_vector_reformat.push_back(h_depth_vector[i]);
        }
    } else if (h_scan_start_loc_.data == "back"){
        for (int i = total_h_scan_points_/2; i < total_h_scan_points_; i++) {
            h_depth_vector_reformat.push_back(h_depth_vector[i]);
        }
        for (int i = 0; i < total_h_scan_points_/2; i++){
            h_depth_vector_reformat.push_back(h_depth_vector[i]);
        }
    } else if (h_scan_start_loc_.data == "left"){
        for (int i = total_h_scan_points_/4; i < total_h_scan_points_; i++) {
            h_depth_vector_reformat.push_back(h_depth_vector[i]);
        }
        for (int i = 0; i < total_h_scan_points_/4; i++){
            h_depth_vector_reformat.push_back(h_depth_vector[i]);
        }
    } else {
        h_depth_vector_reformat = h_depth_vector;
    }

    // Trim the scan down if the entire scan is not being used
    std::vector<float> h_depth_vector_trimmed;
    //ROS_INFO("%d", h_scan_start_index_);
    for(int i=0; i<num_h_scan_points_; i++){
        h_depth_vector_trimmed.push_back(h_depth_vector_reformat[i+h_scan_start_index_]);
    }

    // Check to see if anything has entered the safety boundary
    if(enable_safety_boundary_){
        checkSafetyBoundary(h_depth_vector_trimmed);
    }
/*
     // Publish the reformatted scan
     if(debug_){
        std_msgs::Float32MultiArray h_depth_scan_reformat_msg;
        h_depth_scan_reformat_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        h_depth_scan_reformat_msg.data.clear();
        h_depth_scan_reformat_msg.data = h_depth_vector_trimmed;
        //h_depth_scan_reformat_msg.data = h_depth_vector;
        pub_h_scan_reformat_.publish(h_depth_scan_reformat_msg);
    }

    // Last, convert to cvmat and saturate
    h_depth_cvmat_ = cv::Mat(1,num_h_scan_points_, CV_32FC1);
    std::memcpy(h_depth_cvmat_.data, h_depth_vector_trimmed.data(), h_depth_vector_trimmed.size()*sizeof(float));
    h_depth_cvmat_.setTo(h_sensor_min_dist_, h_depth_cvmat_ < h_sensor_min_dist_);
    h_depth_cvmat_.setTo(h_sensor_max_dist_, h_depth_cvmat_ > h_sensor_max_dist_);
*/
} // End of convertHLaserscan2CVMat

void NearnessController::convertVLaserscan2CVMat(const sensor_msgs::LaserScanPtr v_laserscan_msg){
    std::vector<float> v_depth_vector = v_laserscan_msg->ranges;

    // handle infs due to sensor max distance
    for(int i = 0; i<total_v_scan_points_; i++){
        if(isinf(v_depth_vector[i])){
            v_depth_vector[i] = v_sensor_max_dist_;
        }
    }

    // Reverse the direction
    // Only reverse if the scan point indices are positive ccw
    if(reverse_v_scan_){
        std::reverse(v_depth_vector.begin(), v_depth_vector.end());
    }

    // Reformat the depth scan depending on the orientation of the scanner
    // scan_start_loc describes the location of the first scan index
    std::vector<float> v_depth_vector_reformat;
    v_scan_start_loc_.data = "sim";
    if (v_scan_start_loc_.data == "forward"){
        v_depth_vector_reformat = v_depth_vector;
    } else if (v_scan_start_loc_.data == "right"){
        for (int i = 3*num_v_scan_points_/4; i < num_v_scan_points_; i++) {
            v_depth_vector_reformat.push_back(v_depth_vector[i]);
        }
        for (int i = 0; i < 3*num_v_scan_points_/4; i++){
            v_depth_vector_reformat.push_back(v_depth_vector[i]);
        }
    } else if (v_scan_start_loc_.data == "back"){
        for (int i = num_v_scan_points_/2; i < num_v_scan_points_; i++) {
            v_depth_vector_reformat.push_back(v_depth_vector[i]);
        }
        for (int i = 0; i < num_v_scan_points_/2; i++){
            v_depth_vector_reformat.push_back(v_depth_vector[i]);
        }
    } else if (v_scan_start_loc_.data == "left"){
        for (int i = num_v_scan_points_/4; i < num_v_scan_points_; i++) {
            v_depth_vector_reformat.push_back(v_depth_vector[i]);
        }
        for (int i = 0; i < num_v_scan_points_/4; i++){
            v_depth_vector_reformat.push_back(v_depth_vector[i]);
        }
    } else {
        v_depth_vector_reformat = v_depth_vector;
    }


    // Trim the scan down if the entire scan is not being used
    std::vector<float> v_depth_vector_trimmed;
    for(int i=0; i<num_v_scan_points_; i++){
        v_depth_vector_trimmed.push_back(v_depth_vector_reformat[i+v_scan_start_index_]);
    }

     // Publish the reformatted scan
     if(debug_){
        std_msgs::Float32MultiArray v_depth_scan_reformat_msg;
        v_depth_scan_reformat_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        v_depth_scan_reformat_msg.data.clear();
        v_depth_scan_reformat_msg.data = v_depth_vector_trimmed;
        pub_v_scan_reformat_.publish(v_depth_scan_reformat_msg);
    }

    // Last, convert to cvmat and saturate
    v_depth_cvmat_ = cv::Mat(1,num_v_scan_points_, CV_32FC1);
    std::memcpy(v_depth_cvmat_.data, v_depth_vector_trimmed.data(), v_depth_vector_trimmed.size()*sizeof(float));
    v_depth_cvmat_.setTo(v_sensor_min_dist_, v_depth_cvmat_ < v_sensor_min_dist_);
    v_depth_cvmat_.setTo(v_sensor_max_dist_, v_depth_cvmat_ > v_sensor_max_dist_);

} // End of convertVLaserscan2CVMat

void NearnessController::computeHorizFourierCoeffs(){
    float h_cos_gamma_arr[h_num_fourier_terms_ + 1][num_h_scan_points_];
    float h_sin_gamma_arr[h_num_fourier_terms_ + 1][num_h_scan_points_];

    // Compute horizontal nearness
    h_nearness_ = cv::Mat::zeros(cv::Size(1, num_h_scan_points_), CV_32FC1);
    h_nearness_ = 1.0/ h_depth_cvmat_;

    // Publish horizontal nearness
    if(debug_){
        std::vector<float> h_nearness_array(h_nearness_.begin<float>(), h_nearness_.end<float>());
        std_msgs::Float32MultiArray h_nearness_msg;
        h_nearness_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        h_nearness_msg.layout.dim[0].size = h_nearness_array.size();
        h_nearness_msg.data.clear();
        h_nearness_msg.data.insert(h_nearness_msg.data.end(), h_nearness_array.begin(), h_nearness_array.end());
        pub_h_scan_nearness_.publish(h_nearness_msg);
    }

    // Compute the Fourier Coefficients
    cv::Mat h_cos_gamma_mat(h_num_fourier_terms_ + 1, num_h_scan_points_, CV_32FC1, h_cos_gamma_arr);
    cv::Mat h_sin_gamma_mat(h_num_fourier_terms_ + 1, num_h_scan_points_, CV_32FC1, h_sin_gamma_arr);

    for (int i = 0; i < h_num_fourier_terms_ + 1; i++) {
        for (int j = 0; j < num_h_scan_points_; j++) {
            h_cos_gamma_arr[i][j] = cos(i * h_gamma_vector_[j]);
            h_sin_gamma_arr[i][j] = sin(i * h_gamma_vector_[j]);
        }
        h_a_[i] = h_nearness_.dot(h_cos_gamma_mat.row(i)) * h_dg_ / M_PI;
        h_b_[i] = h_nearness_.dot(h_sin_gamma_mat.row(i)) * h_dg_ / M_PI;
    }

        // Publish horizontal WFI Fourier coefficients
        // Convert array to vector
        std::vector<float> h_a_vector(h_a_, h_a_ + sizeof h_a_ / sizeof h_a_[0]);
        std::vector<float> h_b_vector(h_b_, h_b_ + sizeof h_b_ / sizeof h_b_[0]);

        nearness_control::FourierCoefsMsg h_fourier_coefs_msg;

        h_fourier_coefs_msg.header.stamp = ros::Time::now();
        h_fourier_coefs_msg.a = h_a_vector;
        h_fourier_coefs_msg.b = h_b_vector;

        pub_h_fourier_coefficients_.publish(h_fourier_coefs_msg);
} // End of computeHorizFourierCoeffs

void NearnessController::computeVertFourierCoeffs(){
    float v_cos_gamma_arr[v_num_fourier_terms_ + 1][num_v_scan_points_];
    float v_sin_gamma_arr[v_num_fourier_terms_ + 1][num_v_scan_points_];

    // Compute horizontal nearness
    v_nearness_ = cv::Mat::zeros(cv::Size(1, num_v_scan_points_), CV_32FC1);
    v_nearness_ = 1.0/ v_depth_cvmat_;

    // Publish horizontal nearness
    if(debug_){
        std::vector<float> v_nearness_array(v_nearness_.begin<float>(), v_nearness_.end<float>());
        std_msgs::Float32MultiArray v_nearness_msg;
        v_nearness_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        v_nearness_msg.layout.dim[0].size = v_nearness_array.size();
        v_nearness_msg.data.clear();
        v_nearness_msg.data.insert(v_nearness_msg.data.end(), v_nearness_array.begin(), v_nearness_array.end());
        pub_v_scan_nearness_.publish(v_nearness_msg);
    }

    // Compute the Fourier Coefficients
    cv::Mat v_cos_gamma_mat(v_num_fourier_terms_ + 1, num_v_scan_points_, CV_32FC1, v_cos_gamma_arr);
    cv::Mat v_sin_gamma_mat(v_num_fourier_terms_ + 1, num_v_scan_points_, CV_32FC1, v_sin_gamma_arr);

    for (int i = 0; i < v_num_fourier_terms_ + 1; i++) {
        for (int j = 0; j < num_v_scan_points_; j++) {
            v_cos_gamma_arr[i][j] = cos(i * v_gamma_vector_[j]);
            v_sin_gamma_arr[i][j] = sin(i * v_gamma_vector_[j]);
        }
        v_a_[i] = v_nearness_.dot(v_cos_gamma_mat.row(i)) * v_dg_ / M_PI;
        v_b_[i] = v_nearness_.dot(v_sin_gamma_mat.row(i)) * v_dg_ / M_PI;
    }

        // Publish horizontal WFI Fourier coefficients
        // Convert array to vector
        std::vector<float> v_a_vector(v_a_, v_a_ + sizeof v_a_ / sizeof v_a_[0]);
        std::vector<float> v_b_vector(v_b_, v_b_ + sizeof v_b_ / sizeof v_b_[0]);

        nearness_control::FourierCoefsMsg v_fourier_coefs_msg;

        v_fourier_coefs_msg.header.stamp = ros::Time::now();
        v_fourier_coefs_msg.a = v_a_vector;
        v_fourier_coefs_msg.b = v_b_vector;

        pub_v_fourier_coefficients_.publish(v_fourier_coefs_msg);
} // End of computeVertFourierCoeffs

void NearnessController::computeSFYawRateCommand(){
    std::vector<float> h_nearness(h_nearness_.begin<float>(), h_nearness_.end<float>());
    std::vector<float> recon_wf_nearness;
    std::vector<float> h_sf_nearness;

    // Reconstruct the WF signal
    for(int i = 0; i < num_h_scan_points_; i++){
        recon_wf_nearness[i] = 0.0;
        for(int n = 0; n < h_num_fourier_terms_; n++){
            recon_wf_nearness[i] += h_a_[n+1]*cos((n+1)*h_gamma_vector_[i]) + h_b_[n+1]*sin((n+1)*h_gamma_vector_[i]);
        }
        recon_wf_nearness[i] += h_a_[0]/2.0;
    }

    // Remove the reoconstructed WF signal from the measured nearness signal
    float h_sf_mean_sum = 0.0;
    float h_sf_mean_val = 0.0;
    for(int i=0; i < num_h_scan_points_; i++){
        h_sf_nearness[i] = abs(h_nearness[i] - recon_wf_nearness[i]);
        h_sf_mean_sum += h_sf_nearness[i];
    }

    // Compute the standard deviation of the SF signal
    h_sf_mean_val = h_sf_mean_sum / num_h_scan_points_;
    float h_sf_std_dev = 0.0;
    for (int i= 0; i < num_h_scan_points_; i++){
        h_sf_std_dev += pow((h_sf_nearness[i] - h_sf_mean_val), 2);
    }
    h_sf_std_dev = pow(h_sf_std_dev / num_h_scan_points_, .5);
    float h_sf_min_threshold = h_sf_k_thresh_ * h_sf_std_dev;

    // Find the max value of the signal and determine if it is greaster
    // than the dynamic threshold
    int h_sf_max_val_index = std::max_element(h_sf_nearness.begin(), h_sf_nearness.end()) - h_sf_nearness.begin();
    float h_sf_max_val = *std::max_element(h_sf_nearness.begin(), h_sf_nearness.end());

    float d_0 = abs(h_sf_nearness[h_sf_max_val_index]);
    float r_0 = h_gamma_vector_[h_sf_max_val_index];

    // Compute the sf steering control command
    if(d_0 > h_sf_min_threshold && ((r_0 > (-M_PI/2 + .01)) && (r_0 < (M_PI - .01)))){
        h_sf_r_cmd_ = h_sf_k_0_ * sgn(r_0) * exp(-h_sf_k_psi_ * abs(r_0)) * exp(-h_sf_k_d_/abs(d_0));
    }

    // Publish sf nearness signal
    if(debug_){
        std_msgs::Float32 h_sf_cmd_msg;
        h_sf_cmd_msg.data = h_sf_r_cmd_;
        pub_h_sf_yawrate_command_.publish(h_sf_cmd_msg);

        std_msgs::Float32MultiArray h_sf_nearness_msg;
        h_sf_nearness_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        h_sf_nearness_msg.layout.dim[0].size = h_sf_nearness.size();
        h_sf_nearness_msg.data.clear();
        h_sf_nearness_msg.data.insert(h_sf_nearness_msg.data.end(), h_sf_nearness.begin(), h_sf_nearness.end());
        pub_h_sf_nearness_.publish(h_sf_nearness_msg);

        std_msgs::Float32MultiArray recon_wf_nearness_msg;
        recon_wf_nearness_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        recon_wf_nearness_msg.layout.dim[0].size = recon_wf_nearness.size();
        recon_wf_nearness_msg.data.clear();
        recon_wf_nearness_msg.data.insert(recon_wf_nearness_msg.data.end(), recon_wf_nearness.begin(), recon_wf_nearness.end());
        pub_h_recon_wf_nearness_.publish(recon_wf_nearness_msg);
    }
}

void NearnessController::computeSFVerticalSpeedCommand(){
    std::vector<float> v_nearness(v_nearness_.begin<float>(), v_nearness_.end<float>());
    std::vector<float> recon_wf_nearness;
    std::vector<float> v_sf_nearness;

    // Reconstruct the WF signal
    for(int i = 0; i < num_v_scan_points_; i++){
        recon_wf_nearness[i] = 0.0;
        for(int n = 0; n < v_num_fourier_terms_; n++){
            recon_wf_nearness[i] += v_a_[n+1]*cos((n+1)*v_gamma_vector_[i]) + v_b_[n+1]*sin((n+1)*v_gamma_vector_[i]);
        }
        recon_wf_nearness[i] += v_a_[0]/2.0;
    }

    // Remove the reoconstructed WF signal from the measured nearness signal
    float v_sf_mean_sum = 0.0;
    float v_sf_mean_val = 0.0;
    for(int i=0; i < num_v_scan_points_; i++){
        v_sf_nearness[i] = abs(v_nearness[i] - recon_wf_nearness[i]);
        v_sf_mean_sum += v_sf_nearness[i];
    }

    // Compute the standard deviation of the SF signal
    v_sf_mean_val = v_sf_mean_sum / num_v_scan_points_;
    float v_sf_std_dev = 0.0;
    for (int i= 0; i < num_v_scan_points_; i++){
        v_sf_std_dev += pow((v_sf_nearness[i] - v_sf_mean_val), 2);
    }
    v_sf_std_dev = pow(v_sf_std_dev / num_v_scan_points_, .5);
    float v_sf_min_threshold = v_sf_k_thresh_ * v_sf_std_dev;

    // Find the max value of the signal and determine if it is greaster
    // than the dynamic threshold
    int v_sf_max_val_index = std::max_element(v_sf_nearness.begin(), v_sf_nearness.end()) - v_sf_nearness.begin();
    float v_sf_max_val = *std::max_element(v_sf_nearness.begin(), v_sf_nearness.end());

    float d_0 = abs(v_sf_nearness[v_sf_max_val_index]);
    float r_0 = v_gamma_vector_[v_sf_max_val_index];

    // Compute the sf steering control command
    if(d_0 > v_sf_min_threshold && ((r_0 > (-M_PI/2 + .01)) && (r_0 < (M_PI - .01)))){
        v_sf_w_cmd_ = v_sf_k_0_ * sgn(r_0) * exp(-v_sf_k_psi_ * abs(r_0)) * exp(-v_sf_k_d_/abs(d_0));
    }

    // Publish sf nearness signal
    if(debug_){
        std_msgs::Float32 v_sf_cmd_msg;
        v_sf_cmd_msg.data = v_sf_w_cmd_;
        pub_v_sf_vertspeed_command_.publish(v_sf_cmd_msg);

        std_msgs::Float32MultiArray v_sf_nearness_msg;
        v_sf_nearness_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        v_sf_nearness_msg.layout.dim[0].size = v_sf_nearness.size();
        v_sf_nearness_msg.data.clear();
        v_sf_nearness_msg.data.insert(v_sf_nearness_msg.data.end(), v_sf_nearness.begin(), v_sf_nearness.end());
        pub_v_sf_nearness_.publish(v_sf_nearness_msg);

        std_msgs::Float32MultiArray recon_wf_nearness_msg;
        recon_wf_nearness_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        recon_wf_nearness_msg.layout.dim[0].size = recon_wf_nearness.size();
        recon_wf_nearness_msg.data.clear();
        recon_wf_nearness_msg.data.insert(recon_wf_nearness_msg.data.end(), recon_wf_nearness.begin(), recon_wf_nearness.end());
        pub_v_recon_wf_nearness_.publish(recon_wf_nearness_msg);
    }
}

void NearnessController::computeForwardSpeedCommand(){

    u_cmd_ = u_max_ * (1 - u_k_hb_1_*abs(h_b_[1]) - u_k_hb_2_*abs(h_b_[2]) - u_k_vb_1_*abs(v_b_[1]) - u_k_vb_2_*abs(v_b_[2]));

    // Saturate forward velocity command
    if(u_cmd_ < u_min_){
        u_cmd_ = u_min_;
    } else if (u_cmd_ > u_max_){
        u_cmd_ = u_max_;
    }
} // End of computeForwardSpeedCommand

void NearnessController::computeWFYawRateCommand(){

    h_wf_r_cmd_ = r_k_hb_1_*h_b_[1] + r_k_hb_2_*h_b_[2];
    // Saturate the wide field yaw rate command
    if(h_wf_r_cmd_ < -r_max_) {
        h_wf_r_cmd_ = -r_max_;
    } else if(h_wf_r_cmd_ > r_max_) {
        h_wf_r_cmd_ = r_max_;
    }
} // End of computeWFYawRateCommand

void NearnessController::computeAttractorCommand(){
    if(have_attractor_){
        float attractor_x_pos = next_waypoint_pos_.x;
        float attractor_y_pos = next_waypoint_pos_.y;
        //ROS_INFO_THROTTLE(.5,"x: %f, y: %f", current_pos_.x, current_pos_.y);
        float attractor_d = sqrt(pow((current_pos_.x - attractor_x_pos), 2) + pow((current_pos_.y - attractor_y_pos), 2));
        float relative_attractor_heading = atan2((attractor_y_pos - current_pos_.y),(attractor_x_pos - current_pos_.x));

        attractor_yaw_cmd_ = r_k_att_0_*wrapAngle(current_heading_ - relative_attractor_heading)*exp(-r_k_att_d_*attractor_d);
        //float attractor_yaw_cmd = r_k_att_0_*wrapAngle(current_heading_ - relative_attractor_heading)*exp(-r_k_att_d_*attractor_d);
        //ROS_INFO_THROTTLE(.5,"del_t: %1.2f, e: %1.2f, yaw_cmd: %1.2f", wrapAngle(current_heading_ - relative_attractor_heading), exp(-r_k_att_d_*attractor_d), attractor_yaw_cmd);

    } else {
        attractor_yaw_cmd_ = 0.0;
    }
}

void NearnessController::computeLateralSpeedCommand(){
    h_wf_v_cmd_ = v_k_hb_1_*h_b_[1];
    if(h_wf_v_cmd_ < -v_max_) {
        h_wf_v_cmd_ = -v_max_;
    } else if(h_wf_v_cmd_ > v_max_) {
        h_wf_v_cmd_ = v_max_;
    }
}

void NearnessController::computeWFVerticalSpeedCommand(){

    v_wf_w_cmd_ = w_k_1_*v_b_[1] + w_k_2_*v_b_[2];
    // Saturate the wide field vertical speed command
    if(v_wf_w_cmd_ < -w_max_) {
        v_wf_w_cmd_ = -w_max_;
    } else if(v_wf_w_cmd_ > w_max_) {
        v_wf_w_cmd_ = w_max_;
    }
} // End of computeWFYawRateCommand

void NearnessController::publishControlCommandMsg(){

    control_command_.header.stamp = ros::Time::now();
    control_command_.twist.linear.x = u_cmd_;
    control_command_.twist.linear.y = 0.0;
    control_command_.twist.linear.z = 0.0;

    control_command_.twist.angular.x = 0.0;
    control_command_.twist.angular.y = 0.0;
    control_command_.twist.angular.z = 0.0;

    if(enable_wf_control_){
        control_command_.twist.linear.y = h_wf_v_cmd_;
        control_command_.twist.linear.z = v_wf_w_cmd_;
        control_command_.twist.angular.z = h_wf_r_cmd_;

    }

    //control_command_.twist.linear.z = -.5*(range_agl_ - 2.0);
    if(enable_sf_control_){
        control_command_.twist.angular.z += h_sf_r_cmd_;
        control_command_.twist.linear.z += v_sf_w_cmd_;
    }

    if(enable_attractor_control_){
        control_command_.twist.angular.z += attractor_yaw_cmd_;
        control_command_.twist.linear.x = .25;
    }

    if(is_ground_vehicle_){
        control_command_.twist.linear.z = 0.0;
        control_command_.twist.linear.y = 0.0;
    }

    if(!have_attractor_ && enable_attractor_control_){
        control_command_.twist.linear.x = 0.0;
    }

    saturateControls();

    if(flag_estop_){
        control_command_.twist.linear.x = 0.0;
        control_command_.twist.linear.y = 0.0;
        control_command_.twist.linear.z = 0.0;

        control_command_.twist.angular.x = 0.0;
        control_command_.twist.angular.y = 0.0;
        control_command_.twist.angular.z = 0.0;
    }

    pub_control_commands_.publish(control_command_);

    geometry_msgs::Twist sim_cmd_msg;
    sim_cmd_msg = control_command_.twist;
    pub_sim_control_commands_.publish(sim_cmd_msg);

}

void NearnessController::odomCb(const nav_msgs::OdometryConstPtr& odom_msg){
    current_pos_ = odom_msg->pose.pose.position;
    geometry_msgs::Quaternion vehicle_quat_msg = odom_msg->pose.pose.orientation;
    tf::Quaternion vehicle_quat_tf;
    tf::quaternionMsgToTF(vehicle_quat_msg, vehicle_quat_tf);
    tf::Matrix3x3(vehicle_quat_tf).getRPY(current_roll_, current_pitch_, current_heading_);
}

void NearnessController::sonarHeightCb(const sensor_msgs::RangeConstPtr& range_msg){
    range_agl_ = range_msg->range;
}

void NearnessController::joyconCb(const sensor_msgs::JoyConstPtr& joy_msg)
{
    if((joy_msg->buttons[2] == 1)){
        flag_estop_ = false;
        ROS_INFO_THROTTLE(2,"Controller ESTOP disengaged");

    }
	  if(joy_msg->buttons[1] == 1){
        flag_estop_ = true;
	      ROS_INFO_THROTTLE(2,"Controller ESTOP engaged");
    }

    if(joy_msg->buttons[7] == 1){
        ROS_INFO_THROTTLE(1,"Disable estop");
        std_msgs::Bool engage_msg;
        engage_msg.data = false;
        pub_estop_engage_.publish(engage_msg);
    }
    if(joy_msg->buttons[6] == 1){
        ROS_INFO_THROTTLE(1,"Enable estop");
        std_msgs::Bool engage_msg;
        engage_msg.data = true;
        pub_estop_engage_.publish(engage_msg);
    }
}

void NearnessController::nextWaypointCb(const geometry_msgs::PointStampedConstPtr& next_waypoint_msg){
    ROS_INFO("Received waypoint");
    next_waypoint_pos_ = next_waypoint_msg->point;
    if (!have_attractor_){
        have_attractor_ = true;
    }
}

void NearnessController::generateSafetyBox(){
    ROS_INFO("Generating safety box.");
    bool safety_box_corner_switch = false;
    // Generate the left boundary
    for(int i = 0; i < num_h_scan_points_/2; i++){
        if(h_dg_*i < atan(f_dist_/s_dist_)){
            safety_boundary_.push_back(s_dist_/cos(h_dg_*float(i)));
        } else {
            if(!safety_box_corner_switch){
                safety_box_corner_switch = true;
                left_corner_index_ = i;
            }
            safety_boundary_.push_back(f_dist_/cos(M_PI/2 - h_dg_*float(i)));
        }
    }
    // Copy the left side to the right
    for(int i = 0; i < num_h_scan_points_/2; i++){
        safety_boundary_.push_back(safety_boundary_[num_h_scan_points_/2 - i]);
    }
}

void NearnessController::checkSafetyBoundary(std::vector<float> scan){
    flag_too_close_ = false;

    for(int i = 0; i < num_h_scan_points_; i++){
        if((scan[i] < safety_boundary_[i]) && (scan[i] > h_sensor_min_noise_)){
            if((i <= left_corner_index_) || (i >= (num_h_scan_points_ - left_corner_index_))) {
                flag_too_close_ = true;
            } else {
                flag_too_close_ = flag_too_close_ || flag_too_close_;
            }
        }
    }
}

float NearnessController::sgn(double v) {
    return (v < 0.0) ? -1.0 : ((v > 0.0) ? 1.0 : 0.0);
}

void NearnessController::saturateControls(){
    if(control_command_.twist.angular.z > r_max_){
        control_command_.twist.angular.z = r_max_;
    } else if (control_command_.twist.angular.z < -r_max_){
        control_command_.twist.angular.z = -r_max_;
    }
}

float NearnessController::wrapAngle(float angle){
    if (angle > M_PI){
        angle -= 2*M_PI;
    } else if( angle < -M_PI){
        angle += 2*M_PI;
    }

    return angle;

}

 // end of class
} // End of namespace nearness
