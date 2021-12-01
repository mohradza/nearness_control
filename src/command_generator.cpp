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
    state_sub_ = nh_.subscribe("state", 1, &commandGenerator::stateCb, this);

    // PUBLISHERS
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_out", 10);

    state_ = "init";

    starting_point_.x = 0.0;
    starting_point_.y = 0.0;
    starting_point_.z = 0.75;

    routine_ = "doublets";
    start_doublets_ = false;
    doublet_period_ = 2.0;
    doublet_amplitude_ = 1.0;

  }

  void commandGenerator::stateCb(const std_msgs::String state_msg){
    state_ = state_msg.data;
  }

  void commandGenerator::publishControlCommands(){
    generateCommandVel();
    pub_cmd_vel_.publish(cmd_vel_);
  }

  void commandGenerator::generateCommandVel(){
    cmd_vel_msg_.linear.x = 0.0;
    cmd_vel_msg_.linear.y = 0.0;
    cmd_vel_msg_.linear.z = 0.0;

    cmd_vel_msg_.angular.x = 0.0;
    cmd_vel_msg_.angular.y = 0.0;
    cmd_vel_msg_.angular.z = 0.0;

    if(!state_.compare("init")){
      // Do nothing
    } else if(!state_.compare("go_to_start")){
      // Go to the starting point
    } else if(!state_.compare("start_test")){
      // Start the test routine
      if(!routine_.compare("doublets")){
        generateDoubletsCommand();
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
    }

    if(!first_doublet_complete_){
      // First doublet
      if(time_diff_s <= doublet_period_/2.0){
        cmd_vel_msg_.linear.y = doublet_amplitude_;
      } else if(time_diff_s >= doublet_period_/2.0){
        cmd_vel_msg_.linear.y = -doublet_amplitude_;
      }
    } else {
      // Second doublet
      if(time_diff_s <= doublet_period_/4.0){
        cmd_vel_msg_.linear.y = doublet_amplitude_;
      } else if(time_diff_s >= doublet_period_/4.0){
        cmd_vel_msg_.linear.y = -doublet_amplitude_;
      }
    }
  }


} // end node namespace
