#include <nearness_control/command_generator.h>

namespace command_generator{
commandGenerator::commandGenerator(const ros::NodeHandle &node_handle,
                                       const ros::NodeHandle &private_node_handle)
      :nh_(node_handle),
       pnh_(private_node_handle){
      this->init();
  }

  void commandGenerator::init(){
    // SUBSCRIBERS
    sub_state_ = nh_.subscribe("state", 1, &commandGenerator::stateCb, this);
    sub_odom_ = nh_.subscribe("odometry", 1, &commandGenerator::odomCb, this);

    // PUBLISHERS
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_out", 10);

    state_ = "init";

    starting_point_.x = 0.0;
    starting_point_.y = 0.0;
    starting_point_.z = 1.5;
    starting_heading_ = 0.0;

    goal_point_.x = 0.0;
    goal_point_.y = 1.0;
    goal_point_.z = 1.5;
    goal_heading_ = 1.0;

    routine_ = "doublets";
    routine_ = "dynamic";
    // routine_ = "const";
    start_doublets_ = false;
    doublet_period_ = 8.0;
    doublet_amplitude_ = 1.0;

    k_u_ = 0.75;
    k_v_ = 1.0;
    k_w_ = 1.5;
    k_r_ = 0.5;

    xv_k_ = 0.0;


    // Mv_Xk_ << 0.0, 0.0, 0.0, 0.0, 0.0;
    Mv_Xk_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    Mr_Xk_ << 0.0, 0.0, 0.0, 0.0, 0.0;

  // Lateral - Mixed synthesis, 4x4 state model, 2 inputs
  Mv_A_ <<     1.0000,   -0.0000,   -0.0000,    0.0000,    0.0000,    0.0000,    0.0000,   -0.0000,    0.0000,    0.0000,
               0.0000,    0.9851,    0.0000,   -0.0000,   -0.0000,   -0.0000,   -0.0000,    0.0000,   -0.0000,   -0.0000,
               0.4686,   -0.0003,    0.7534,   -0.0367,   -0.0467,   -0.0545,   -1.1439,   -0.0031,   -0.0031,   -0.0015,
               0.3290,   -0.0002,    0.2845,    0.8047,   -0.1200,   -0.0770,   -0.8030,   -0.0022,   -0.0022,   -0.0010,
               0.0250,   -0.0000,    0.0247,    0.1443,    0.9904,   -0.0061,   -0.0610,   -0.0002,   -0.0002,   -0.0001,
               0.0006,   -0.0000,    0.0007,    0.0060,    0.0797,    0.9998,   -0.0016,   -0.0000,   -0.0000,   -0.0000,
               0.0000,   -0.0000,    0.0000,    0.0000,    0.0001,    0.0025,    1.0000,   -0.0000,   -0.0000,   -0.0000,
               0.1645,   -0.0001,    0.1423,   -0.0129,   -0.0164,   -0.0191,   -0.4015,    0.8293,   -0.0883,   -0.0392,
               0.0125,   -0.0000,    0.0124,   -0.0010,   -0.0012,   -0.0014,   -0.0305,    0.1461,    0.9928,   -0.0032,
               0.0003,   -0.0000,    0.0003,   -0.0000,   -0.0000,   -0.0000,   -0.0008,    0.0060,    0.0798,    0.9999;

  Mv_B_ <<     0.0100,    0.0000,
              -0.0000,    0.0099,
               0.0025,   -0.0000,
               0.0016,   -0.0000,
               0.0001,   -0.0000,
               0.0000,   -0.0000,
              -0.0000,    0.0000,
               0.0008,   -0.0000,
               0.0000,   -0.0000,
               0.0000,   -0.0000;

  Mv_C_ <<     3.3970,   -0.0024,    4.5351,   -0.2622,   -0.3348,  -0.3904,   -8.2916,   -0.0230,   -0.0230,   -0.0111;

  // Heading - Mixed sythesis, 2x2 state model, 2 inputs
  // Mr_A_ <<     0.9999,   -0.0000,   -0.0000,   -0.0000,   -0.0000,
  //              0.0000,    0.9802,   -0.0000,   -0.0000,   -0.0000,
  //              1.7231,   -0.2134,    0.4934,   -0.1714,   -3.6133,
  //              2.4167,   -0.2997,    0.1878,    0.5383,   -5.0789,
  //              0.0254,   -0.0032,    0.0023,    0.0152,    0.9465;
  //
  // Mr_B_ <<     0.0200,   -0.0000,
  //             -0.0000,    0.0198,
  //              0.0203,   -0.0025,
  //              0.0254,   -0.0032,
  //              0.0002,   -0.0000;
  //
  // Mr_C_ <<    34.3496,   -4.3046,    4.3365,   -3.0566,  -72.0497;

  // Heading - Mixed sythesis, 2x2 state model, 2 inputs, no modifications
  Mr_A_ <<         0.9999,    0.0000,    0.0000,    0.0000,    0.0000,
                  -0.0000,    0.9891,    0.0000,    0.0000,    0.0000,
                   1.2226,   -0.0904,    0.4649,   -0.1669,   -3.1719,
                   1.1753,   -0.0870,    0.3851,    0.6177,   -3.0605,
                   0.0112,   -0.0008,    0.0047,    0.0162,    0.9709;

  Mr_B_ <<         0.0200,   -0.0000,
                   0.0000,    0.0099,
                   0.0141,   -0.0005,
                   0.0112,   -0.0004,
                   0.0001,   -0.0000;

  Mr_C_ <<       11.4841,   -0.8545,    8.3533,   -1.4462,  -29.8040;
;





  }

  void commandGenerator::stateCb(const std_msgs::String state_msg){
    state_ = state_msg.data;
  }

  void commandGenerator::odomCb(const nav_msgs::OdometryConstPtr& odom_msg){
      current_odom_ = *odom_msg;
      current_pos_ = current_odom_.pose.pose.position;
      //current_odom_.pose.pose.position.z = current_height_agl_;
      geometry_msgs::Quaternion vehicle_quat_msg = current_odom_.pose.pose.orientation;
      tf::Quaternion vehicle_quat_tf;
      tf::quaternionMsgToTF(vehicle_quat_msg, vehicle_quat_tf);
      tf::Matrix3x3(vehicle_quat_tf).getRPY(current_roll_, current_pitch_, current_heading_);

      p_ = current_odom_.twist.twist.angular.x;
      r_ = current_odom_.twist.twist.angular.z;
  }

  void commandGenerator::publishControlCommands(){
    generateCommandVel();
    pub_cmd_vel_.publish(cmd_vel_msg_);
  }

  void commandGenerator::generateCommandVel(){
    // ROS_INFO_THROTTLE(1.0,"state: %s", state_.c_str());
    cmd_vel_msg_.linear.x = 0.0;
    cmd_vel_msg_.linear.y = 0.0;
    cmd_vel_msg_.linear.z = 0.0;

    cmd_vel_msg_.angular.x = 0.0;
    cmd_vel_msg_.angular.y = 0.0;
    cmd_vel_msg_.angular.z = 0.0;

    if(!state_.compare("init")){
      // Do nothing
      start_doublets_ = false;
    } else if(!state_.compare("go_to_start")){
      // Go to the starting point
      cmd_vel_msg_.linear.x = k_u_*(starting_point_.x - current_pos_.x);
      cmd_vel_msg_.linear.z = k_w_*(starting_point_.z - current_pos_.z);
      cmd_vel_msg_.linear.y = k_v_*(starting_point_.y - current_pos_.y);
      cmd_vel_msg_.angular.z = (k_r_/50.0)*(starting_heading_ - current_heading_);



    } else if(!state_.compare("standby")){
      // Do nothing, just wait
    } else if(!state_.compare("start_test")){
      // Start the test routine

      if(!routine_.compare("const")){
        cmd_vel_msg_.linear.x = k_u_*(starting_point_.x - current_pos_.x);
        cmd_vel_msg_.linear.z = k_w_*(starting_point_.z - current_pos_.z);
        // cmd_vel_msg_.linear.y = 1.0;
        // cmd_vel_msg_.linear.y = k_v_*(starting_point_.y - current_pos_.y);
        // cmd_vel_msg_.angular.z = (k_r_/50.0)*(starting_heading_ - current_heading_);
        cmd_vel_msg_.angular.z = 2.0;
      }

      if(!routine_.compare("doublets")){
        generateDoubletsCommand();
        cmd_vel_msg_.linear.x = k_u_*(starting_point_.x - current_pos_.x);
        // cmd_vel_msg_.linear.z = k_w_*(starting_point_.z - current_pos_.z);
        cmd_vel_msg_.linear.y = k_v_*(starting_point_.y - current_pos_.y);
        cmd_vel_msg_.angular.z = (k_r_/50.0)*(starting_heading_ - current_heading_);
      }

      if(!routine_.compare("dynamic")){
        cmd_vel_msg_.linear.x = k_u_*(starting_point_.x - current_pos_.x);
        cmd_vel_msg_.linear.y = k_v_*(starting_point_.y - current_pos_.y);
        cmd_vel_msg_.linear.z = k_w_*(starting_point_.z - current_pos_.z);
        // cmd_vel_msg_.angular.z = (k_r_/50.0)*(starting_heading_ - current_heading_);

        u_y_ = (goal_point_.y - current_pos_.y);
        // Complex lateral dynamic controller
        // Vector2f a(u_y_, -p_);
        // Mv_Xkp1_ = Mv_A_*Mv_Xk_ + Mv_B_*a;
        // u_v_ = k_v_*Mv_C_*Mv_Xkp1_;
        // Mv_Xk_ = Mv_Xkp1_;
        // ROS_INFO("u_v: %f", u_v_);

        float u_psi = (goal_heading_ - current_heading_);
        // Complex heading dynamic controller
        Vector2f a(u_psi, -r_);
        Mr_Xkp1_ = Mr_A_*Mr_Xk_ + Mr_B_*a;
        u_r_ = Mr_C_*Mr_Xkp1_;
        Mr_Xk_ = Mr_Xkp1_;
        ROS_INFO("u_r: %f, psi: %f", u_r_, u_psi);

        if(isnan(u_r_)){
          state_ = "init";
        }

        cmd_vel_msg_.linear.y = u_v_;
        cmd_vel_msg_.angular.z = u_r_;

      }

    }

    if(cmd_vel_msg_.linear.x > 2.0){
      cmd_vel_msg_.linear.x = 2.0;
    }

  }

  void commandGenerator::generateDoubletsCommand(){
    if(!start_doublets_){
      start_doublets_ = true;
      start_doublets_time_ = ros::Time::now();
      first_doublet_complete_ = false;
    }

    float time_diff_s = (ros::Time::now() - start_doublets_time_).toSec();
    if(time_diff_s > doublet_period_ && !first_doublet_complete_){
      first_doublet_complete_ = true;
      start_doublets_time_ = ros::Time::now();
      time_diff_s = (ros::Time::now() - start_doublets_time_).toSec();
    }

    float cmd;
    if(!first_doublet_complete_){
      // First doublet
      if(time_diff_s <= doublet_period_/4.0){
        cmd = doublet_amplitude_;
      } else if((time_diff_s >= doublet_period_/4.0) && (time_diff_s <= doublet_period_/2.0)){
        cmd = -doublet_amplitude_;
      } else if((time_diff_s >= doublet_period_/2.0) && (time_diff_s <= 3.0*doublet_period_/4.0)){
        cmd = doublet_amplitude_;
      } else if((time_diff_s >= 3.0*doublet_period_/4.0) && (time_diff_s <= doublet_period_)){
        cmd = -doublet_amplitude_;
      }

    } else {
      // Second doublet
      if(time_diff_s <= doublet_period_/8.0){
        cmd = doublet_amplitude_;
      } else if((time_diff_s >= doublet_period_/8.0) && (time_diff_s <= doublet_period_/4.0)){
        cmd = -doublet_amplitude_;
      } else if((time_diff_s >= doublet_period_/4.0) && (time_diff_s <= 3.0*doublet_period_/8.0)){
        cmd = doublet_amplitude_;
      } else if((time_diff_s >= 3.0*doublet_period_/8.0) && (time_diff_s <= doublet_period_/2.0)){
        cmd = -doublet_amplitude_;
      } else if(time_diff_s >= doublet_period_/2.0){
        state_ = "init";
      }
    }

    // cmd_vel_msg_.linear.y = cmd;
    cmd_vel_msg_.linear.z = cmd;
    // cmd_vel_msg_.angular.z = cmd;
  }


} // end node namespace
