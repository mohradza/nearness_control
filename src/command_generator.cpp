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

    starting_point_.x = 4.0;
    starting_point_.y = 0.0;
    starting_point_.z = 0.75;
    starting_heading_ = 0.0;

    routine_ = "doublets";
    start_doublets_ = false;
    doublet_period_ = 8.0;
    doublet_amplitude_ = 0.5;

    k_u_ = 0.75;
    k_v_ = 0.75;
    k_w_ = 1.5;
    k_r_ = 0.5;

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
  }

  void commandGenerator::publishControlCommands(){
    generateCommandVel();
    pub_cmd_vel_.publish(cmd_vel_msg_);
  }

  void commandGenerator::generateCommandVel(){
    ROS_INFO_THROTTLE(1.0,"state: %s", state_.c_str());
    cmd_vel_msg_.linear.x = 0.0;
    cmd_vel_msg_.linear.y = 0.0;
    cmd_vel_msg_.linear.z = 0.0;

    cmd_vel_msg_.angular.x = 0.0;
    cmd_vel_msg_.angular.y = 0.0;
    cmd_vel_msg_.angular.z = 0.0;

    if(!state_.compare("init")){
      // Do nothing
      start_doublets_ = false;
      ROS_INFO_THROTTLE(1.0,"z pos: %f", current_pos_.z);
    } else if(!state_.compare("go_to_start")){
      // Go to the starting point
      cmd_vel_msg_.linear.x = k_u_*(starting_point_.x - current_pos_.x);
      cmd_vel_msg_.linear.z = k_w_*(starting_point_.z - current_pos_.z);
      cmd_vel_msg_.linear.y = k_v_*(starting_point_.y - current_pos_.y);
      cmd_vel_msg_.angular.z = (k_r_/50.0)*(starting_heading_ - current_heading_);

    } else if(!state_.compare("start_test")){
      // Start the test routine
      if(!routine_.compare("doublets")){
        generateDoubletsCommand();
        cmd_vel_msg_.linear.z = k_w_*(starting_point_.z - current_pos_.z);
      }
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

    cmd_vel_msg_.linear.y = cmd;
    //cmd_vel_msg_.linear.z = cmd;
    //cmd_vel_msg_.angular.z = cmd;
  }


} // end node namespace
