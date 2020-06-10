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

    // Set up subscribers and callbacks
    sub_horiz_laserscan_ = nh_.subscribe("horiz_scan", 1, &NearnessController::horizLaserscanCb, this);
    sub_odom_ = nh_.subscribe("odometry", 1, &NearnessController::odomCb, this);
    sub_imu_ = nh_.subscribe("imu", 1, &NearnessController::imuCb, this);
    sub_joy_ = nh_.subscribe("joy", 1, &NearnessController::joyconCb, this);
    subt_enable_control_ = nh_.subscribe("enable_control", 1, &NearnessController::enableControlCb, this);
    sub_next_waypoint_ = nh_.subscribe("next_waypoint", 1, &NearnessController::nextWaypointCb, this);

    pub_h_scan_reformat_ = nh_.advertise<std_msgs::Float32MultiArray>("horiz_depth_reformat", 10);
    pub_h_scan_nearness_ = nh_.advertise<std_msgs::Float32MultiArray>("horiz_nearness", 10);
    pub_h_sf_nearness_ = nh_.advertise<std_msgs::Float32MultiArray>("horiz_sf_nearness", 10);
    pub_h_recon_wf_nearness_ = nh_.advertise<std_msgs::Float32MultiArray>("horiz_recon_wf_nearness", 10);
    pub_h_fourier_coefficients_ = nh_.advertise<nearness_control::FourierCoefsMsg>("horiz_fourier_coefficients", 10);
    pub_h_sf_yawrate_command_ = nh_.advertise<std_msgs::Float32>("sf_yawrate_command", 10);
    pub_control_commands_ = nh_.advertise<geometry_msgs::Twist>("control_commands", 10);
    //pub_debug_weighting_ = nh_.advertise<std_msgs::Float32MultiArray>("debug_weighting", 10);
    pub_vehicle_status_ = nh_.advertise<std_msgs::Int32>("vehicle_status", 10);
    pub_estop_engage_ = nh_.advertise<std_msgs::Bool>("estop_cmd", 10);
    pub_sf_clustering_debug_ = nh_.advertise<nearness_control::ClusterMsg>("sf_clusters", 10);

    // Initialize global variables
    have_attractor_ = false;
    lost_attractor_ = false;
    flag_estop_ = true;
    control_command_.header.frame_id = "/base_stabilized";
    flag_safety_too_close_ = false;
    x_vel_filt_last_ = 0.0;
    r_vel_filt_last_ = 0.0;
    h_wf_r_filt_last_ = 0.0;
    att_angle_error_ = 0.0;
    enable_attitude_limits_ = false;

    // Import parameters
    pnh_.param("enable_debug", debug_, false);
    pnh_.param("enable_wf_control", enable_wf_control_, true);
    pnh_.param("enable_sf_control", enable_sf_control_, false);
    pnh_.param("enable_attractor_control", enable_attractor_control_, false);
    pnh_.param("enable_command_weighting", enable_command_weighting_, false);
    pnh_.param("enable_sf_clustering", enable_sf_clustering_, false);
    pnh_.param("enable_att_speed_reg", enable_att_speed_reg_, false);
    pnh_.param("stagger_waypoints_", stagger_waypoints_, false);
    pnh_.param("enable_tower_safety", enable_tower_safety_, false);
    pnh_.param("enable_cmd_lp_filter", enable_cmd_lp_filter_, true);
    pnh_.param("motion_on_startup", motion_on_startup_, false);

    if(motion_on_startup_){
      flag_estop_ = false;
    }

    pnh_.param("total_horiz_scan_points", total_h_scan_points_, 1440);
    pnh_.param("horiz_scan_limit", h_scan_limit_, M_PI);
    pnh_.param("num_horiz_scan_points", num_h_scan_points_, 720);
    pnh_.param("horiz_sensor_min_distance", h_sensor_min_dist_, .1);
    pnh_.param("horiz_sensor_max_distance", h_sensor_max_dist_, 25.0);
    pnh_.param("horiz_scan_start_index", h_scan_start_index_, 0);
    pnh_.param("horiz_sensor_min_noise", h_sensor_min_noise_ , .1);
    pnh_.param("reverse_horiz_scan", reverse_h_scan_, true);
    pnh_.param("num_horiz_fourier_terms", h_num_fourier_terms_, 5);

    // Safety
    pnh_.param("enable_safety_boundary", enable_safety_boundary_, false);
    pnh_.param("enable_safety_box", enable_safety_box_, false);
    pnh_.param("enable_safety_radius", enable_safety_radius_, false);
    pnh_.param("front_safety_distance", f_dist_, .5);
    pnh_.param("side_safety_distance", s_dist_, .5);
    pnh_.param("safety_radius", safety_radius_, .5);
    pnh_.param("close_side_speed", close_side_speed_, .05);
    pnh_.param("reverse_forward_speed_cmd", reverse_u_cmd_, false);
    pnh_.param("safety_getting_close_vote_thresh", safety_getting_close_vote_thresh_, 10);
    pnh_.param("safety_too_close_vote_thresh", safety_too_close_vote_thresh_, 6);

    // Wide Field Forward Speed and Yaw Rate Control Gains
    pnh_.param("forward_speed_k_hb_1", u_k_hb_1_, 0.0);
    pnh_.param("forward_speed_k_hb_2", u_k_hb_2_, 0.0);
    pnh_.param("forward_speed_k_ha_1", u_k_ha_1_, 0.0);
    pnh_.param("forward_speed_k_ha_2", u_k_ha_2_, 0.0);
    pnh_.param("forward_speed_k_att", u_k_att_, 0.0);
    pnh_.param("forward_speed_min", u_min_, .1);
    pnh_.param("forward_speed_max", u_max_, 5.0);
    pnh_.param("yaw_rate_k_hb_1", r_k_hb_1_, 2.0);
    pnh_.param("yaw_rate_k_hb_2", r_k_hb_2_, 2.0);
    pnh_.param("yaw_rate_max", r_max_, 1.0);
    pnh_.param("reverse_yaw_rate_cmd", reverse_r_cmd_, false);
    pnh_.param("reverse_wf_yaw_rate_cmd", reverse_wf_r_cmd_, false);

    // Small Field Yaw Rate Control Gains
    pnh_.param("h_sf_k_0", h_sf_k_0_, 0.5);
    pnh_.param("h_sf_k_d", h_sf_k_d_, 1.0);
    pnh_.param("h_sf_k_psi", h_sf_k_psi_, 0.4);
    pnh_.param("h_sf_k_thresh", h_sf_k_thresh_, 3.0);

    // Attractor Control Gains
    pnh_.param("yaw_rate_k_att_0", r_k_att_0_, 1.0);
    pnh_.param("yaw_rate_k_att_d_", r_k_att_d_, 0.1);
    pnh_.param("yaw_rate_k_att_turn", r_k_att_turn_, 0.1);
    pnh_.param("attractor_latch_thresh", attractor_latch_thresh_, 1.0);
    pnh_.param("attractor_watchdog_timer", attractor_watchdog_timer_, 3.0);

    // Filtering
    pnh_.param("forward_speed_lp_filter_alpha", alpha_x_vel_, 1.0);
    pnh_.param("yaw_rate_lp_filter_alpha", alpha_r_vel_, 1.0);
    pnh_.param("wf_yaw_rate_lp_filter_alpha", alpha_h_wf_r_, 1.0);
    pnh_.param("sf_yaw_rate_lp_filter_alpha", alpha_h_sf_r_, 1.0);
    //ROS_INFO("u_a: %f, r_a: %f, wf_a: %f, sf_a: %f", alpha_x_vel_, alpha_r_vel_, alpha_h_wf_r_, alpha_h_sf_r_);

    // Vehicle status
    pnh_.param("vehicle_roll_angle_limit", roll_limit_, 15.0);
    pnh_.param("vehicle_pitch_angle_limit", pitch_limit_, 15.0);

    // Generate the horizontal gamma vector
    for(int i=0; i<num_h_scan_points_; i++){
        h_gamma_vector_.push_back((float(i)/float(num_h_scan_points_))*(2*h_scan_limit_) - h_scan_limit_);
    }
    h_dg_ = (2.0*h_scan_limit_)/num_h_scan_points_;

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
    u_k_ha_1_ = config_.forward_speed_k_ha_1;
    u_k_ha_2_ = config_.forward_speed_k_ha_2;
    u_k_vb_1_ = config_.forward_speed_k_vb_1;
    u_k_vb_2_ = config_.forward_speed_k_vb_2;
    u_k_att_ = config_.forward_speed_k_att;
    u_min_ = config_.forward_speed_min;
    u_max_ = config_.forward_speed_max;

    r_k_hb_1_ = config_.yaw_rate_k_hb_1;
    r_k_hb_2_ = config_.yaw_rate_k_hb_2;
    r_max_ = config_.yaw_rate_max;

    r_k_att_0_ = config.yaw_rate_k_att_0;
    r_k_att_d_ = config.yaw_rate_k_att_d;
    r_k_att_turn_ = config.yaw_rate_k_turn;

    h_sf_k_thresh_ = config_.h_sf_k_thresh;
    h_sf_k_0_ = config_.h_sf_k_0;
    h_sf_k_psi_ = config_.h_sf_k_psi;
    h_sf_k_d_ = config_.h_sf_k_d;

}

void NearnessController::horizLaserscanCb(const sensor_msgs::LaserScanPtr h_laserscan_msg){

    // Convert incoming scan to cv matrix and reformat
    convertHLaserscan2CVMat(h_laserscan_msg);

    // Compute the Fourier harmonics of the signal
    computeHorizFourierCoeffs();

    // Feed back fourier coefficients for forward speed regulation
    computeForwardSpeedCommand();

    computeWFYawRateCommand();

    computeSFYawRateCommand();

    if(enable_attractor_control_){
        computeAttractorCommand();
    } else {
        attractor_yaw_cmd_ = 0.0;
    }

    publishControlCommandMsg();

}

void NearnessController::convertHLaserscan2CVMat(const sensor_msgs::LaserScanPtr h_laserscan_msg){
    std::vector<float> h_depth_vector = h_laserscan_msg->ranges;
    std::vector<float> h_depth_vector_noinfs = h_depth_vector;

    // handle infs due to sensor max distance
    for(int i = 0; i<total_h_scan_points_; i++){
        if(isinf(h_depth_vector[i])){
            //h_depth_vector[i] = h_sensor_max_dist_;
            if(i == 0){
                if(isinf(h_depth_vector[i+1])){
                    h_depth_vector_noinfs[i] = h_sensor_max_dist_;
                } else {
                    h_depth_vector_noinfs[i] = h_depth_vector[i+1];
                }
            } else if(i == total_h_scan_points_){
                if(isinf(h_depth_vector[i-1])){
                    h_depth_vector_noinfs[i] = h_sensor_max_dist_;
                } else {
                    h_depth_vector_noinfs[i] = h_depth_vector[i-1];
                }
            } else {
                if(isinf(h_depth_vector[i-1]) && isinf(h_depth_vector[i+1])){
                    h_depth_vector_noinfs[i] = h_sensor_max_dist_;
                } else {
                    h_depth_vector_noinfs[i] = (h_depth_vector[i-1] + h_depth_vector[i+1])/2.0;
                }
            }
        }
        h_depth_vector = h_depth_vector_noinfs;
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

} // End of convertHLaserscan2CVMat

void NearnessController::computeHorizFourierCoeffs(){
    float h_cos_gamma_arr[h_num_fourier_terms_ + 1][num_h_scan_points_];
    float h_sin_gamma_arr[h_num_fourier_terms_ + 1][num_h_scan_points_];


    // Compute horizontal nearness
    h_nearness_ = cv::Mat::zeros(cv::Size(1, num_h_scan_points_), CV_32FC1);
    h_nearness_ = 1.0/ h_depth_cvmat_;

    std::vector<float> h_nearness_array(h_nearness_.begin<float>(), h_nearness_.end<float>());
    h_nearness_maxval_ = *std::max_element(h_nearness_array.begin(), h_nearness_array.end());

    // Pull out the L2 norm of the measured nearness signal
    float norm_sum = 0.0;
    //std::accumulate(h_depth_vector_reformat.begin(), h_depth_vector_reformat.end
    for(int i=0; i<num_h_scan_points_; i++){
        norm_sum += pow(h_nearness_array[i],2);
    }
    h_nearness_l2_norm_ = sqrt(norm_sum);
    //ROS_INFO_THROTTLE(1,"maxval: %f, norm: %f", h_nearness_maxval_, h_nearness_l2_norm_);

    // Publish horizontal nearness
    if(debug_){
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

void NearnessController::computeSFYawRateCommand(){
    std::vector<float> h_nearness(h_nearness_.begin<float>(), h_nearness_.end<float>());
    std::vector<float> recon_wf_nearness(num_h_scan_points_, 0.0);
    std::vector<float> h_sf_nearness(num_h_scan_points_, 0.0);

    // Reconstruct the WF signal
    for(int i = 0; i < num_h_scan_points_; i++){
        //recon_wf_nearness.push_back(0.0);
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
        //h_sf_mean_sum += h_sf_nearness[i];
    }

    // Band-aid for bad sf edges
    // Trim the ends off the sf_signal
    int edge_band = 35;
    std::vector<float> h_sf_nearness_trimmed;
    for (int i = edge_band; i < (num_h_scan_points_ - edge_band); i++){
        h_sf_nearness_trimmed.push_back(h_sf_nearness[i]);
    }
    int num_trimmed_points = num_h_scan_points_-(2*edge_band);

    // Compute the standard deviation of the SF signal
    h_sf_mean_val = h_sf_mean_sum / float(num_trimmed_points);
    float h_sf_std_dev = 0.0;
    for (int i= 0; i < num_trimmed_points; i++){
        h_sf_std_dev += pow((h_sf_nearness_trimmed[i] - h_sf_mean_val), 2);
    }
    h_sf_std_dev = pow(h_sf_std_dev / num_trimmed_points, .5);
    float h_sf_min_threshold = h_sf_k_thresh_ * h_sf_std_dev;
    //ROS_INFO("%f, %f", h_sf_min_threshold, h_sf_std_dev);

    if(!enable_sf_clustering_){
        // Find the max value of the signal and determine if it is greater
        // than the dynamic threshold
        int h_sf_max_val_index = std::max_element(h_sf_nearness.begin(), h_sf_nearness.end()) - h_sf_nearness.begin();
        float h_sf_max_val = *std::max_element(h_sf_nearness.begin(), h_sf_nearness.end());

        float d_0 = abs(h_sf_nearness[h_sf_max_val_index]);
        float r_0 = h_gamma_vector_[h_sf_max_val_index];

        // Compute the sf steering control command
        if(d_0 > h_sf_min_threshold && ((r_0 > (-M_PI/2 + .01)) && (r_0 < (M_PI - .01)))){
            h_sf_r_cmd_ = h_sf_k_0_ * sgn(r_0) * exp(-h_sf_k_psi_ * abs(r_0)) * exp(-h_sf_k_d_/abs(d_0));
        }
    } else {
        // Do clustering and mixing
        std::vector<float> sf_d_cluster;
        std::vector<float> sf_r_cluster;
        std::vector<float> cluster_d(200, 0.0);
        std::vector<float> cluster_r(200, 0.0);
        int n = 0;
        int c = 0;
        num_sf_clusters_ = 0;
        //h_sf_min_threshold = .5;
        for(int i=0; i < num_trimmed_points; i++){
            if((h_sf_nearness_trimmed[i] > h_sf_min_threshold) && (h_sf_nearness_trimmed[i+1] > h_sf_min_threshold)){
                sf_d_cluster.push_back(h_sf_nearness_trimmed[i]);
                sf_r_cluster.push_back(h_gamma_vector_[i+edge_band]);
                n++;
            } else {
                if (n > 0){
                    //cluster_d.push_back(0.0);
                    //cluster_r.push_back(0.0);
                    for(int j = 0; j < n; j++){
                        cluster_d[c] += sf_d_cluster[j];
                        cluster_r[c] += sf_r_cluster[j];
                    }
                    sf_d_cluster.clear();
                    sf_r_cluster.clear();
                    cluster_d[c] /= float(n);
                    cluster_r[c] /= float(n);
                    c++;
                    n = 0;
                }
            }
        }
        num_sf_clusters_ = c;
        h_sf_r_cmd_ = 0.0;
        int sign = 1;

        if(num_sf_clusters_ != 0){
            for(int i = 0; i < num_sf_clusters_; i++){
                if(cluster_r[i] < 0) sign = 1;
                if(cluster_r[i] > 0) sign = -1;
                h_sf_r_cmd_ += -h_sf_k_0_*float(sign)*exp(-h_sf_k_psi_*abs(cluster_r[i]))*exp(-h_sf_k_d_/abs(cluster_d[i]));
            }
        }

        if(debug_){
            nearness_control::ClusterMsg cluster_msg;
            cluster_msg.num_clusters = num_sf_clusters_;
            if(num_sf_clusters_ != 0){
                for(int i = 0; i < num_sf_clusters_; i++){
                    cluster_msg.cluster_mag.push_back(cluster_d[i]);
                    cluster_msg.cluster_loc.push_back(cluster_r[i]);
                }
            }
            pub_sf_clustering_debug_.publish(cluster_msg);
        }
    }


    // Publish sf nearness signal
    if(debug_){
        std_msgs::Float32 h_sf_cmd_msg;
        h_sf_cmd_msg.data = h_sf_r_cmd_;
        pub_h_sf_yawrate_command_.publish(h_sf_cmd_msg);

        std_msgs::Float32MultiArray h_sf_nearness_msg;
        h_sf_nearness_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        h_sf_nearness_msg.layout.dim[0].size = h_sf_nearness_trimmed.size();
        h_sf_nearness_msg.data.clear();
        h_sf_nearness_msg.data.insert(h_sf_nearness_msg.data.end(), h_sf_nearness_trimmed.begin(), h_sf_nearness_trimmed.end());
        pub_h_sf_nearness_.publish(h_sf_nearness_msg);

        std_msgs::Float32MultiArray recon_wf_nearness_msg;
        recon_wf_nearness_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        recon_wf_nearness_msg.layout.dim[0].size = recon_wf_nearness.size();
        recon_wf_nearness_msg.data.clear();
        recon_wf_nearness_msg.data.insert(recon_wf_nearness_msg.data.end(), recon_wf_nearness.begin(), recon_wf_nearness.end());
        pub_h_recon_wf_nearness_.publish(recon_wf_nearness_msg);

    }
}

void NearnessController::computeForwardSpeedCommand(){
    float angle_error = wrapAngle(relative_attractor_heading_ - current_heading_);
    if(!enable_att_speed_reg_ || !enable_attractor_control_ || lost_attractor_){
        angle_error = 0.0;
    }
    if(enable_wf_control_){
        u_cmd_ = u_max_ * (1 - u_k_hb_1_*abs(h_b_[1]) - u_k_hb_2_*abs(h_b_[2]) - u_k_hb_1_*abs(h_a_[1]) - u_k_hb_2_*abs(h_a_[2]) -abs(h_sf_r_cmd_) - u_k_att_*abs(angle_error));
        //ROS_INFO_THROTTLE(1, "%f %f", u_k_hb_1_, u_k_hb_2_);
        //u_cmd_ = u_max_ * (1 - u_k_hb_1_*abs(h_b_[1]) - u_k_hb_2_*abs(h_b_[2]));
    } else {
        u_cmd_ = u_max_*(1 - u_k_att_*abs(angle_error));
    }
    // Saturate forward velocity command
    if(u_cmd_ < u_min_){
        u_cmd_ = u_min_;
    } else if (u_cmd_ > u_max_){
        u_cmd_ = u_max_;
    }
} // End of computeForwardSpeedCommand

void NearnessController::computeWFYawRateCommand(){
    if(reverse_wf_r_cmd_){
        h_wf_r_cmd_ = -1*(r_k_hb_1_*h_b_[1] + r_k_hb_2_*h_b_[2]);
    } else {
        h_wf_r_cmd_ = r_k_hb_1_*h_b_[1] + r_k_hb_2_*h_b_[2];
    }
    // Saturate the wide field yaw rate command
    if(h_wf_r_cmd_ < -r_max_) {
        h_wf_r_cmd_ = -r_max_;
    } else if(h_wf_r_cmd_ > r_max_) {
        h_wf_r_cmd_ = r_max_;
    }
} // End of computeWFYawRateCommand

void NearnessController::computeAttractorCommand(){

    float attractor_timer = (ros::Time::now() - last_wp_msg_time_).toSec();
    //ROS_INFO_THROTTLE(1, "attractor_timer: %f, timer_limit: %f", attractor_timer, attractor_watchdog_timer_);
    if(attractor_timer > attractor_watchdog_timer_){
        ROS_INFO_THROTTLE(1,"Have not received a new attractor for %f seconds.", attractor_timer);
        lost_attractor_ = true;
    } else {
        lost_attractor_ = false;
    }

    att_angle_error_ = wrapAngle(relative_attractor_heading_ - current_heading_);
    //ROS_INFO("att_err: %f, current: %f", att_angle_error_, current_heading_);
    //float angle_error_backup = wrapAngle(relative_attractor_heading_ - wrapAngle(current_heading_ - M_PI));
    //backup_attractor_yaw_cmd_ = r_k_att_0_*angle_error_backup*exp(-r_k_att_d_*attractor_d_);
    //ROS_INFO_THROTTLE(1,"backup yaw cmd: %f", backup_attractor_yaw_cmd_);
    if(have_attractor_ && (abs(att_angle_error_) < 1.4)){
        attractor_yaw_cmd_ = r_k_att_0_*att_angle_error_*exp(-r_k_att_d_*attractor_d_);
        //attractor_yaw_cmd_ = -r_k_att_d_*attractor_d_;
        //ROS_INFO("%f, %f, %f, %f, %f", r_k_att_0_,r_k_att_d_, attractor_d_, att_angle_error_, attractor_yaw_cmd_);
        //float attractor_yaw_cmd = r_k_att_0_*wrapAngle(current_heading_ - relative_attractor_heading)*exp(-r_k_att_d_*attractor_d);
        //ROS_INFO_THROTTLE(.5,"del_t: %1.2f, e: %1.2f, yaw_cmd: %1.2f", wrapAngle(current_heading_ - relative_attractor_heading), exp(-r_k_att_d_*attractor_d), attractor_yaw_cmd);
        attractor_turn_ = false;
    } else {
	      ROS_INFO_THROTTLE(1,"Pure attractor turn");
        attractor_yaw_cmd_ = r_k_att_turn_*att_angle_error_;
        //attractor_yaw_cmd_ = .05;
	      attractor_turn_ = true;
        //u_cmd_ = 0.0;
    }
    if(isnan(attractor_yaw_cmd_)){
        attractor_yaw_cmd_ = 0.0;
    }
}

void NearnessController::publishControlCommandMsg(){

    control_command_.header.stamp = ros::Time::now();
    control_command_.twist.linear.x = u_cmd_;
    control_command_.twist.linear.y = 0.0;
    control_command_.twist.linear.z = 0.0;

    control_command_.twist.angular.x = 0.0;
    control_command_.twist.angular.y = 0.0;
    control_command_.twist.angular.z = 0.0;

    if(enable_cmd_lp_filter_){
        float h_wf_r_filt = 0.0;
        float h_wf_r_prefilt = h_wf_r_cmd_;
        h_wf_r_filt = alpha_h_wf_r_*h_wf_r_prefilt + (1 - alpha_h_wf_r_)*h_wf_r_filt_last_;
        //ROS_INFO("x_vel_prefilt: %f, x_vel_last: %f, x_vel_post: %f", x_vel_prefilt,  x_vel_filt_last_, x_vel_filt);
        h_wf_r_filt_last_ = h_wf_r_filt;
        h_wf_r_cmd_ = h_wf_r_filt;

        float h_sf_r_filt = 0.0;
        float h_sf_r_prefilt = h_sf_r_cmd_;
        h_sf_r_filt = alpha_h_sf_r_*h_sf_r_prefilt + (1 - alpha_h_sf_r_)*h_sf_r_filt_last_;
        //ROS_INFO("x_vel_prefilt: %f, x_vel_last: %f, x_vel_post: %f", x_vel_prefilt,  x_vel_filt_last_, x_vel_filt);
        h_sf_r_filt_last_ = h_sf_r_filt;
        h_sf_r_cmd_ = h_sf_r_filt;
    }

    if(enable_wf_control_){
        control_command_.twist.angular.z = h_wf_r_cmd_;
    }

    if(enable_sf_control_){
        control_command_.twist.angular.z += h_sf_r_cmd_;
    }

    if(enable_attractor_control_ && !lost_attractor_){
        control_command_.twist.angular.z += attractor_yaw_cmd_;
        if(attractor_turn_){
            control_command_.twist.linear.x = 0.0;
            control_command_.twist.angular.z = sat(attractor_yaw_cmd_, -r_max_, r_max_);
          }
    }

    // Wait for an attractor before moving
    if(!have_attractor_ && enable_attractor_control_){
        control_command_.twist.linear.x = 0.0;
    }

    if(debug_){
        float nearness_r_cmd = 0.0;
        if (enable_wf_control_){
            nearness_r_cmd += h_wf_r_cmd_;
        }

        if (enable_sf_control_){
            nearness_r_cmd += h_sf_r_cmd_;
        }
        std_msgs::Float32MultiArray weighting_msg;
        weighting_msg.data.push_back(nearness_r_cmd);
        weighting_msg.data.push_back(attractor_yaw_cmd_);
        weighting_msg.data.push_back(control_command_.twist.angular.z);
        weighting_msg.data.push_back(h_nearness_l2_norm_);
        weighting_msg.data.push_back(h_nearness_maxval_);
        //pub_debug_weighting_.publish(weighting_msg);
    }

    if(enable_cmd_lp_filter_){
        float x_vel_filt = 0.0;
        float x_vel_prefilt = control_command_.twist.linear.x;
        x_vel_filt = alpha_x_vel_*x_vel_prefilt + (1 - alpha_x_vel_)*x_vel_filt_last_;
        //ROS_INFO("x_vel_prefilt: %f, x_vel_last: %f, x_vel_post: %f", x_vel_prefilt,  x_vel_filt_last_, x_vel_filt);
        x_vel_filt_last_ = x_vel_filt;
        control_command_.twist.linear.x = x_vel_filt;

        float r_vel_filt = 0.0;
        float r_vel_prefilt = control_command_.twist.angular.z;
        r_vel_filt = alpha_r_vel_*control_command_.twist.angular.z + (1 - alpha_r_vel_)*r_vel_filt_last_;
        //ROS_INFO("r_vel_prefilt: %f, r_vel_last: %f, r_vel_post: %f", r_vel_prefilt,  r_vel_filt_last_, r_vel_filt);
        r_vel_filt_last_ = r_vel_filt;
        control_command_.twist.angular.z = r_vel_filt;
    }

    // If something is too close on either side, we should move slowly
    if(flag_too_close_side_){
      control_command_.twist.linear.x = close_side_speed_;
      ROS_INFO_THROTTLE(1,"Too close! RPLidar side!");
    }

    // If we are getting close to hitting something with the ouster
    // we should move slowly as we approach it.
    if(flag_safety_getting_close_ && enable_tower_safety_){
      control_command_.twist.linear.x = close_side_speed_;
      ROS_INFO_THROTTLE(1,"Tower safety: getting close!");
    }

    // Saturate the outgoing control command
    saturateControls();

    if(reverse_r_cmd_){
        control_command_.twist.angular.z = -1*control_command_.twist.angular.z;
    }

    if(reverse_u_cmd_){
        control_command_.twist.linear.x = -1*control_command_.twist.linear.x;
    }

    if(flag_too_close_front_){
      ROS_INFO_THROTTLE(1,"Too close in the front! Lidar: %s", (flag_too_close_front_ ? "true" : "false"));
      control_command_.twist.linear.x = 0.0;
    }

    if(flag_safety_too_close_ && enable_tower_safety_){
      ROS_INFO_THROTTLE(1,"Tower safety: too close!");
      control_command_.twist.linear.x = 0.0;
    }
    ROS_INFO_THROTTLE(1, "WF: %f, SF: %f, ATT: %f, YAWMIX: %f, U: %f", h_wf_r_cmd_, h_sf_r_cmd_, attractor_yaw_cmd_, control_command_.twist.angular.z, control_command_.twist.linear.x);

    // Estop goes last so it overrides everything.
    if(flag_estop_){
        control_command_.twist.linear.x = 0.0;
        control_command_.twist.linear.y = 0.0;
        control_command_.twist.linear.z = 0.0;

        control_command_.twist.angular.x = 0.0;
        control_command_.twist.angular.y = 0.0;
        control_command_.twist.angular.z = 0.0;
    }

    //pub_control_commands_stamped_.publish(control_command_);
    pub_control_commands_.publish(control_command_.twist);

}

void NearnessController::odomCb(const nav_msgs::OdometryConstPtr& odom_msg){
    current_pos_ = odom_msg->pose.pose.position;
    geometry_msgs::Quaternion vehicle_quat_msg = odom_msg->pose.pose.orientation;
    tf::Quaternion vehicle_quat_tf;
    tf::quaternionMsgToTF(vehicle_quat_msg, vehicle_quat_tf);
    tf::Matrix3x3(vehicle_quat_tf).getRPY(current_roll_, current_pitch_, current_heading_);
}

void NearnessController::joyconCb(const sensor_msgs::JoyConstPtr& joy_msg)
{

    // if(joy_msg->buttons[7] == 1){
    //     ROS_INFO_THROTTLE(1,"Disable estop");
    //     std_msgs::Bool engage_msg;
    //     engage_msg.data = false;
    //     pub_estop_engage_.publish(engage_msg);
    // }
    // if(joy_msg->buttons[6] == 1){
    //     ROS_INFO_THROTTLE(1,"Enable estop");
    //     std_msgs::Bool engage_msg;
    //     engage_msg.data = true;
    //     pub_estop_engage_.publish(engage_msg);
    // }
}

void NearnessController::enableControlCb(const std_msgs::BoolConstPtr& enable_msg){
    flag_estop_ = !enable_msg->data;
    if (!enable_msg->data){
        ROS_INFO("Controller ESTOP engaged.");
    } else {
        ROS_INFO("Controller ESTOP disengaged.");
    }
}

void NearnessController::nextWaypointCb(const geometry_msgs::PointStampedConstPtr& next_waypoint_msg){
    if (!have_attractor_){
        have_attractor_ = true;
    }
    last_wp_msg_time_ = ros::Time::now();

    //ROS_INFO_THROTTLE(2,"Received new ATTRACTOR");

    if(stagger_waypoints_){
        if(attractor_d_ < attractor_latch_thresh_){
          next_waypoint_pos_ = next_waypoint_msg->point;
        }
    } else {
        next_waypoint_pos_ = next_waypoint_msg->point;
    }

    attractor_d_ = sqrt(pow((current_pos_.x - next_waypoint_pos_.x), 2) + pow((current_pos_.y - next_waypoint_pos_.y), 2));
    relative_attractor_heading_ = atan2((next_waypoint_pos_.y - current_pos_.y),(next_waypoint_pos_.x - current_pos_.x));
    //ROS_INFO("%f", relative_attractor_heading_);
}

void NearnessController::towerSafetyCb(const std_msgs::Int32ConstPtr& safety_msg)
{
    if(enable_tower_safety_){
        if(safety_msg->data == 1){
            //flag_safety_too_close = true;
            safety_getting_close_counter_[safety_counter1_] = 1;
            safety_too_close_counter_[safety_counter2_] = 0;
        } else if (safety_msg->data == 2){
            //flag_safety_getting_close = true;
            safety_too_close_counter_[safety_counter2_] = 1;
            safety_getting_close_counter_[safety_counter1_] = 0;
        } else {
            //flag_safety_too_close = false;
            //flag_safety_getting_close = false;
            safety_getting_close_counter_[safety_counter1_] = 0;
            safety_too_close_counter_[safety_counter2_] = 0;
        }

        // Tally the votes
        int total_safety_getting_close_votes = std::accumulate(safety_getting_close_counter_.begin(), safety_getting_close_counter_.end(), 0);
        int total_safety_too_close_votes = std::accumulate(safety_too_close_counter_.begin(), safety_too_close_counter_.end(), 0);
        //ROS_INFO("Getting close votes: %d, Too close votes: %d", total_safety_getting_close_votes, total_safety_too_close_votes);

        if(total_safety_getting_close_votes > safety_getting_close_vote_thresh_){
            flag_safety_getting_close_ = true;
        } else {
            flag_safety_getting_close_ = false;
        }

        if(total_safety_too_close_votes > safety_too_close_vote_thresh_){
            flag_safety_too_close_ = true;
        } else {
            flag_safety_too_close_ = false;
        }

        safety_counter1_++;
        safety_counter2_++;
        if (safety_counter1_ == safety_getting_close_num_votes_){
            safety_counter1_ = 0;
        }
        if (safety_counter2_ == safety_too_close_num_votes_){
            safety_counter2_ = 0;
        }

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
  flag_too_close_front_ = false;
  flag_too_close_side_ = false;

    for(int i = 0; i < num_h_scan_points_; i++){
        if((scan[i] < safety_boundary_[i]) && (scan[i] > h_sensor_min_noise_)){
            if((i <= left_corner_index_) || (i >= (num_h_scan_points_ - left_corner_index_))) {
                flag_too_close_side_ = true;
                //ROS_INFO("Side");
            } else {
                flag_too_close_front_ = true;
                //ROS_INFO("Front");
            }
        } else {
          flag_too_close_front_ = flag_too_close_front_ || flag_too_close_front_;
          flag_too_close_side_ = flag_too_close_side_ || flag_too_close_side_;
        }
    }
}

void NearnessController::imuCb(const sensor_msgs::ImuConstPtr& imu_msg){
    geometry_msgs::Quaternion vehicle_imu_quat_msg = imu_msg->orientation;
    tf::Quaternion vehicle_imu_quat_tf;
    tf::quaternionMsgToTF(vehicle_imu_quat_msg, vehicle_imu_quat_tf);
    tf::Matrix3x3(vehicle_imu_quat_tf).getRPY(roll_, pitch_, imu_yaw_);
    roll_ = wrapAngle(roll_ - M_PI);
    float temp_roll = roll_;
    roll_ = pitch_;
    pitch_ = -temp_roll;
    //ROS_INFO("Roll: %f, Pitch: %f", roll_, pitch_);
    if(((abs(roll_) > roll_limit_) || (abs(pitch_) > pitch_limit_)) && enable_attitude_limits_) {
        flag_safety_attitude_ = true;
    } else {
        flag_safety_attitude_ = false;
    }
}

void NearnessController::checkVehicleStatus(){
    // Check imu
    if(flag_safety_attitude_){
        ROS_INFO_THROTTLE(1,"Vehicle has bad attitude");
    }

    // Check odom

    // Check for stuck

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
    if(control_command_.twist.linear.x > u_max_){
        control_command_.twist.linear.x = u_max_;
    } else if (control_command_.twist.linear.x < u_min_){
        if(attractor_turn_){
            control_command_.twist.linear.x = 0.0;
        } else {
            control_command_.twist.linear.x = u_min_;
        }
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

float NearnessController::sat(float num, float min_val, float max_val){
    if (num >= max_val){
        return max_val;
    } else if( num <= min_val){
         return min_val;
    } else {
      return num;
    }
}

 // end of class
} // End of namespace nearness
