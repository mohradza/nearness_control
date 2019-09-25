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
    /*reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
    ReconfigureServer::CallbackType f = boost::bind(&ScanNodelet::configCb, this, _1, _2);
    reconfigure_server_->setCallback(f);*/


    // Set up subscribers and callbacks
    sub_horiz_laserscan_ = nh_.subscribe("horiz_scan", 1, &NearnessController::horizLaserscanCb, this);
    //sub_odom_ = nh_.subscribe("odometry", 1, &NearnessController::odomCb, this);
    //sub_imu_ = nh_.subscribe("imu_raw", 1, &NearnessController::imuCb, this);
    sub_sonar_height_ = nh_.subscribe("sonar_height", 1, &NearnessController::sonarHeightCb, this);

    // Set up publishers
    pub_h_scan_reformat_ = nh_.advertise<std_msgs::Float32MultiArray>("horiz_depth_reformat", 10);
    pub_h_scan_nearness_ = nh_.advertise<std_msgs::Float32MultiArray>("horiz_nearness", 10);
    pub_h_sf_nearness_ = nh_.advertise<std_msgs::Float32MultiArray>("horiz_sf_nearness", 10);
    pub_h_recon_wf_nearness_ = nh_.advertise<std_msgs::Float32MultiArray>("horiz_recon_wf_nearness", 10);
    //pub_h_fourier_coefficients_ = nh_.advertise<nearness_control::FourierCoefsMsg>("horiz_fourier_coefficients", 10);
    pub_control_commands_ = nh_.advertise<geometry_msgs::Twist>("control_commands", 10);
    pub_sim_control_commands_ = nh_.advertise<geometry_msgs::TwistStamped>("sim_control_commands", 10);
    pub_sf_control_commands_ = nh_.advertise<geometry_msgs::Twist>("sf_control_command", 10);
    pub_vehicle_status_ = nh_.advertise<std_msgs::Int32>("vehicle_status", 10);

    // Import parameters required for init
    nh_.param("nearness_param/total_horiz_scan_points", total_h_scan_points_, 1440);
    nh_.param("nearness_param/num_horiz_scan_points", num_h_scan_points_, 720);
    nh_.getParam("nearness_param/horiz_scan_limit", h_scan_limit_);
    nh_.getParam("nearness_param/enable_safety_boundary", enable_safety_boundary_);
    nh_.getParam("nearness_param/enable_safety_box", enable_safety_box_);
    nh_.getParam("nearness_param/enable_safety_radius", enable_safety_radius_);
    nh_.getParam("nearness_param/front_safety_distance", f_dist_);
    nh_.getParam("wfi_from_depth_sensor_scan/side_safety_distance", s_dist_);
    nh_.param("nearness_param/safety_radius", safety_radius_, .5);

    // Generate the gamma vector
    for(int i=0; i<num_h_scan_points_; i++){
        h_gamma_vector_.push_back((float(i)/float(num_h_scan_points_))*(2*h_scan_limit_) - h_scan_limit_);
    }
    if(enable_safety_boundary_){
        // Generate safety box or radius around vehicle
        if(enable_safety_box_ && !enable_safety_radius_){
            generateSafetyBox();
        } else if(enable_safety_radius_ && !enable_safety_box_){
            for(int i=0; i<num_h_scan_points_; i++){
                safety_boundary_.push_back(safety_radius_);
            }
        } else if(enable_safety_box_ && enable_safety_radius_){
            ROS_INFO("Cannot have safety box and safety radius. Disabling safety boundary.");
            enable_safety_boundary_ = false;
        }
    }
} // End of init
/*
void NearnessController::configCb(Config &config, uint32_t level)
{
    config_ = config;

    //sensor_max_dist_ = config.
    //reverse_h_scan_ = config.

}*/

void NearnessController::horizLaserscanCb(const sensor_msgs::LaserScanPtr h_laserscan_msg){
    convertLaserscan2CVMat(h_laserscan_msg);
}

void NearnessController::convertLaserscan2CVMat(const sensor_msgs::LaserScanPtr h_laserscan_msg){
    std::vector<float> h_depth_vector = h_laserscan_msg->ranges;

    // handle infs due to sensor max distance
    for(int i = 0; i<total_h_scan_points_; i++){
        if(isinf(h_depth_vector[i])){
            h_depth_vector[i] = sensor_max_dist_;
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
    scan_start_loc.data = "back";
    if (scan_start_loc.data == "forward"){
        h_depth_vector_reformat = h_depth_vector;
    } else if (scan_start_loc.data == "right"){
        for (int i = 3*num_h_scan_points_/4; i < num_h_scan_points_; i++) {
            h_depth_vector_reformat.push_back(h_depth_vector[i]);
        }
        for (int i = 0; i < 3*num_h_scan_points_/4; i++){
            h_depth_vector_reformat.push_back(h_depth_vector[i]);
        }
    } else if (scan_start_loc.data == "back"){
        for (int i = num_h_scan_points_/2; i < num_h_scan_points_; i++) {
            h_depth_vector_reformat.push_back(h_depth_vector[i]);
        }
        for (int i = 0; i < num_h_scan_points_/2; i++){
            h_depth_vector_reformat.push_back(h_depth_vector[i]);
        }
    } else if (scan_start_loc.data == "left"){
        for (int i = num_h_scan_points_/4; i < num_h_scan_points_; i++) {
            h_depth_vector_reformat.push_back(h_depth_vector[i]);
        }
        for (int i = 0; i < num_h_scan_points_/4; i++){
            h_depth_vector_reformat.push_back(h_depth_vector[i]);
        }
    }

    // Trim the scan down if the entire scan is not being used
    std::vector<float> h_depth_vector_trimmed;
    for(int i=0; i<num_h_scan_points_; i++){
        h_depth_vector_trimmed.push_back(h_depth_vector_reformat[i+h_scan_start_index_]);
    }

    // Check to see if anything has entered the safety boundary
    checkSafetyBoundary(h_depth_vector_trimmed);

     // Publish the reformatted scan
    std_msgs::Float32MultiArray h_depth_scan_reformat_msg;
    h_depth_scan_reformat_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    h_depth_scan_reformat_msg.data.clear();
    h_depth_scan_reformat_msg.data = h_depth_vector_trimmed;
    pub_h_scan_reformat_.publish(h_depth_scan_reformat_msg);

    // Last, convert to cvmat and saturate
    h_depth_cvmat_ = cv::Mat(1,num_h_scan_points_, CV_32FC1);
    std::memcpy(h_depth_cvmat_.data, h_depth_vector_trimmed.data(), h_depth_vector_trimmed.size()*sizeof(float));
    h_depth_cvmat_.setTo(sensor_min_dist_, h_depth_cvmat_ < sensor_min_dist_);
    h_depth_cvmat_.setTo(sensor_max_dist_, h_depth_cvmat_ > sensor_max_dist_);
} // End of convertLaserscan2CVMat



void NearnessController::sonarHeightCb(const sensor_msgs::RangeConstPtr& range_msg){
    range_agl_ = range_msg->range;
}

void NearnessController::generateSafetyBox(){
    float dg = (M_PI/2)/float(num_h_scan_points_/2.0);
    bool safety_box_corner_switch = false;
    // Generate the left boundary
    for(int i = 0; i < num_h_scan_points_/2; i++){
        if(dg*i < atan(f_dist_/s_dist_)){
            safety_boundary_.push_back(s_dist_/cos(dg*float(i)));
        } else {
            if(!safety_box_corner_switch){
                safety_box_corner_switch = true;
                left_corner_index_ = i;
            }
            safety_boundary_.push_back(f_dist_/cos(M_PI/2 - dg*float(i)));
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
        if((scan[i] < safety_boundary_[i]) && (scan[i] > sensor_min_noise_)){
            if((i <= left_corner_index_) || (i >= (num_h_scan_points_ - left_corner_index_))) {
                flag_too_close_ = true;
            } else {
                flag_too_close_ = flag_too_close_ || flag_too_close_;
            }
        }
    }
}

 // end of class
} // End of namespace nearness
