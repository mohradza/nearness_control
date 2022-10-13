#include <nearness_control/command_generator.h>

namespace command_generator {
commandGenerator::commandGenerator(const ros::NodeHandle &node_handle,
                                   const ros::NodeHandle &private_node_handle)
    : nh_(node_handle), pnh_(private_node_handle) {
  this->init();
}

void commandGenerator::init() {
  // SUBSCRIBERS
  sub_state_ = nh_.subscribe("state", 1, &commandGenerator::stateCb, this);
  sub_odom_ = nh_.subscribe("odometry", 1, &commandGenerator::odomCb, this);

  // PUBLISHERS
  pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_out", 10);
  pub_cmd_vel_stamped_ =
      nh_.advertise<geometry_msgs::TwistStamped>("cmd_vel_out_stamped", 10);

  state_ = "init";

  starting_point_.x = 4.0;
  starting_point_.y = 0.0;
  starting_point_.z = 1.5;
  starting_heading_ = 0.0;

  goal_point_.x = 0.0;
  goal_point_.y = 0.0;
  goal_point_.z = 1.5;
  goal_heading_ = 0.0;

  forward_speed_ = 3.0;

  routine_ = "doublets";
  routine_ = "dynamic";
  routine_ = "const";
  routine_ = "double_const";
  // routine_ = "swerve";
  swerve_dur_ = 4.0;
  start_doublets_ = false;
  doublet_period_ = 8.0;
  doublet_amplitude_ = 1.0;

  k_u_ = 0.75;
  k_v_ = 1.0;
  k_w_ = 2.0;
  k_r_ = 0.5;

  xv_k_ = 0.0;
  c1_ = .2807;
  c2_ = .3743;

  // Mv_Xk_ << 0.0, 0.0, 0.0, 0.0, 0.0;
  Mv_Xk_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  Mr_Xk_ << 0.0, 0.0, 0.0, 0.0;
  Mw_Xk_ << 0.0, 0.0, 0.0, 0.0;

  // Lateral - Mixed synthesis, 4x4 state model, 2 inputs
  Mv_A_ << 1.0000, -0.0000, -0.0000, 0.0000, 0.0000, 0.0000, 0.0000, -0.0000,
      0.0000, 0.0000, 0.0000, 0.9851, 0.0000, -0.0000, -0.0000, -0.0000,
      -0.0000, 0.0000, -0.0000, -0.0000, 0.4686, -0.0003, 0.7534, -0.0367,
      -0.0467, -0.0545, -1.1439, -0.0031, -0.0031, -0.0015, 0.3290, -0.0002,
      0.2845, 0.8047, -0.1200, -0.0770, -0.8030, -0.0022, -0.0022, -0.0010,
      0.0250, -0.0000, 0.0247, 0.1443, 0.9904, -0.0061, -0.0610, -0.0002,
      -0.0002, -0.0001, 0.0006, -0.0000, 0.0007, 0.0060, 0.0797, 0.9998,
      -0.0016, -0.0000, -0.0000, -0.0000, 0.0000, -0.0000, 0.0000, 0.0000,
      0.0001, 0.0025, 1.0000, -0.0000, -0.0000, -0.0000, 0.1645, -0.0001,
      0.1423, -0.0129, -0.0164, -0.0191, -0.4015, 0.8293, -0.0883, -0.0392,
      0.0125, -0.0000, 0.0124, -0.0010, -0.0012, -0.0014, -0.0305, 0.1461,
      0.9928, -0.0032, 0.0003, -0.0000, 0.0003, -0.0000, -0.0000, -0.0000,
      -0.0008, 0.0060, 0.0798, 0.9999;

  Mv_B_ << 0.0100, 0.0000, -0.0000, 0.0099, 0.0025, -0.0000, 0.0016, -0.0000,
      0.0001, -0.0000, 0.0000, -0.0000, -0.0000, 0.0000, 0.0008, -0.0000,
      0.0000, -0.0000, 0.0000, -0.0000;

  Mv_C_ << 3.3970, -0.0024, 4.5351, -0.2622, -0.3348, -0.3904, -8.2916, -0.0230,
      -0.0230, -0.0111;

  // Heading - Mixed sythesis, 2x2 state model, 2 inputs, no modifications
  Mr_A_ << 0.9997, -0.0000, -0.0000, -0.0000, 6.3379, 0.1678, -0.3967, -10.4494,
      4.9941, -0.1875, 0.4815, -8.2398, 0.0341, -0.0017, 0.0068, 0.9437;

  Mr_B_ << 0.0200, 0.0453, 0.0269, 0.0000;

  Mr_C_ << 183.0103, -13.1084, -11.1293, -301.7216;

  // Vertical - Mixed sythesis, 2x2 state model, 1 input, no modifications
  Mw_A_ << 0.9998, 0, 0, 0, 3.0284, 0.1595, -0.2818, -7.3599, 2.0391, -0.3156,
      0.7805, -4.9580, 0.0034, -0.0006, 0.0022, 0.9918;

  Mw_B_ << 0.0200, 0.0298, 0.0177, 0.0000;

  Mw_C_ << 132.8314, -28.3276, -12.0377, -322.7989;
}

void commandGenerator::stateCb(const std_msgs::String state_msg) {
  state_ = state_msg.data;
}

void commandGenerator::odomCb(const nav_msgs::OdometryConstPtr &odom_msg) {
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

void commandGenerator::publishControlCommands() {
  generateCommandVel();
  pub_cmd_vel_.publish(cmd_vel_msg_);

  // Publish a stamped message for time keeping
  geometry_msgs::TwistStamped cmd_vel_stamped_msg;
  cmd_vel_stamped_msg.header.stamp = ros::Time::now();
  cmd_vel_stamped_msg.twist = cmd_vel_msg_;
  pub_cmd_vel_stamped_.publish(cmd_vel_stamped_msg);
}

void commandGenerator::generateCommandVel() {
  // ROS_INFO_THROTTLE(1.0,"state: %s", state_.c_str());
  cmd_vel_msg_.linear.x = 0.0;
  cmd_vel_msg_.linear.y = 0.0;
  cmd_vel_msg_.linear.z = 0.0;

  cmd_vel_msg_.angular.x = 0.0;
  cmd_vel_msg_.angular.y = 0.0;
  cmd_vel_msg_.angular.z = 0.0;

  if (!state_.compare("init")) {
    // Do nothing
    start_doublets_ = false;
    swerve_complete_ = false;
    start_swerve_ = false;
    start_fwd_motion_ = false;
  } else if (!state_.compare("go_to_start")) {
    // Go to the starting point
    cmd_vel_msg_.linear.x = k_u_ * (starting_point_.x - current_pos_.x);
    cmd_vel_msg_.linear.z = k_w_ * (starting_point_.z - current_pos_.z);
    cmd_vel_msg_.linear.y = k_v_ * (starting_point_.y - current_pos_.y);
    cmd_vel_msg_.angular.z =
        (k_r_ / 50.0) * (starting_heading_ - current_heading_);
    ROS_INFO_THROTTLE(0.25, "x: %f, y: %f, z: %f, r: %f", cmd_vel_msg_.linear.x,
                      cmd_vel_msg_.linear.y, cmd_vel_msg_.linear.z,
                      cmd_vel_msg_.angular.z);

  } else if (!state_.compare("standby")) {
    // Do nothing, just wait
  } else if (!state_.compare("start_test")) {
    // Start the test routine

    if (!routine_.compare("const")) {
      // cmd_vel_msg_.linear.x = k_u_*(starting_point_.x - current_pos_.x);
      cmd_vel_msg_.linear.x = 0.5;

      cmd_vel_msg_.linear.y = k_v_ * (starting_point_.y - current_pos_.y);
      // cmd_vel_msg_.linear.y = 0.5;

      cmd_vel_msg_.linear.z = k_w_ * (starting_point_.z - current_pos_.z);
      // cmd_vel_msg_.linear.z = 0.5;

      cmd_vel_msg_.angular.z =
          (k_r_ / 50.0) * (starting_heading_ - current_heading_);
      // cmd_vel_msg_.angular.z = 0.5;
    }

    if (!routine_.compare("double_const")) {
      cmd_vel_msg_.linear.x = forward_speed_;
      // cmd_vel_msg_.linear.x = k_u_*(starting_point_.x - current_pos_.x);
      cmd_vel_msg_.linear.z = k_w_ * (goal_point_.z - current_pos_.z);
      cmd_vel_msg_.linear.y = k_v_ * (goal_point_.y - current_pos_.y);
      // cmd_vel_msg_.linear.y = 0.5;
      cmd_vel_msg_.angular.z =
          (k_r_ / 50.0) * (starting_heading_ - current_heading_);

      generateDoubleConstCommands();

      ROS_INFO_THROTTLE(0.25, "x: %f, y: %f, z: %f, r: %f",
                        cmd_vel_msg_.linear.x, cmd_vel_msg_.linear.y,
                        cmd_vel_msg_.linear.z, cmd_vel_msg_.angular.z);
    }

    if (!routine_.compare("doublets")) {
      generateDoubletsCommand();
      cmd_vel_msg_.linear.x = k_u_ * (starting_point_.x - current_pos_.x);
      // cmd_vel_msg_.linear.z = k_w_*(starting_point_.z - current_pos_.z);
      cmd_vel_msg_.linear.y = k_v_ * (starting_point_.y - current_pos_.y);
      cmd_vel_msg_.angular.z =
          (k_r_ / 50.0) * (starting_heading_ - current_heading_);
    }

    if (!routine_.compare("swerve")) {
      cmd_vel_msg_.linear.x = 1.5;
      cmd_vel_msg_.linear.z = k_w_ * (starting_point_.z - current_pos_.z);
      cmd_vel_msg_.linear.y = k_v_ * (starting_point_.y - current_pos_.y);
      // cmd_vel_msg_.linear.y = 0.0;
      cmd_vel_msg_.angular.z =
          (k_r_ / 50.0) * (starting_heading_ - current_heading_);

      generateSwerveCommands();

      ROS_INFO_THROTTLE(0.25, "x: %f, y: %f, z: %f, r: %f",
                        cmd_vel_msg_.linear.x, cmd_vel_msg_.linear.y,
                        cmd_vel_msg_.linear.z, cmd_vel_msg_.angular.z);
    }

    if (!routine_.compare("dynamic")) {
      cmd_vel_msg_.linear.x = k_u_ * (starting_point_.x - current_pos_.x);
      cmd_vel_msg_.linear.y = k_v_ * (starting_point_.y - current_pos_.y);
      cmd_vel_msg_.linear.z = k_w_ * (starting_point_.z - current_pos_.z);
      cmd_vel_msg_.angular.z =
          (k_r_ / 50.0) * (starting_heading_ - current_heading_);

      u_y_ = (goal_point_.y - current_pos_.y);
      u_z_ = (goal_point_.z - current_pos_.z);
      float u_psi = (goal_heading_ - current_heading_);

      // Complex lateral dynamic controller
      // Vector2f a(u_y_, -p_);
      // Mv_Xkp1_ = Mv_A_*Mv_Xk_ + Mv_B_*a;
      // u_v_ = k_v_*Mv_C_*Mv_Xkp1_;
      // Mv_Xk_ = Mv_Xkp1_;
      // ROS_INFO("u_v: %f", u_v_);

      // Complex heading dynamic controller
      // Vector2f a(u_psi, -r_);
      // Mr_Xkp1_ = Mr_A_*Mr_Xk_ + Mr_B_*a;
      // u_r_ = Mr_C_*Mr_Xkp1_;
      // Mr_Xk_ = Mr_Xkp1_;
      // ROS_INFO("u_r: %f, psi: %f", u_r_, u_psi);

      // Complex vertical dynamic controller
      Mw_Xkp1_ = Mw_A_ * Mw_Xk_ + Mw_B_ * u_z_;
      u_w_ = Mw_C_ * Mw_Xkp1_;
      Mw_Xk_ = Mw_Xkp1_;
      ROS_INFO("u_w: %f, ", u_w_);

      if (isnan(u_w_)) {
        state_ = "init";
      }

      // cmd_vel_msg_.linear.y = u_v_;
      // cmd_vel_msg_.angular.z = u_r_;
      cmd_vel_msg_.linear.z = u_w_;
    }
  }

  // if (cmd_vel_msg_.linear.x > 2.0) {
  //   cmd_vel_msg_.linear.x = 2.0;
  // }
}

void commandGenerator::generateDoubleConstCommands() {
  if (!start_fwd_motion_) {
    fwd_motion_time_ = ros::Time::now();
    start_fwd_motion_ = true;
    start_swerve_ = false;
  }

  float durr = (ros::Time::now() - fwd_motion_time_).toSec();
  bool forward_motion_steady = false;
  if (durr > 10.0) {
    forward_motion_steady = true;
  }

  if (forward_motion_steady) {
    cmd_vel_msg_.linear.x = forward_speed_;
    // cmd_vel_msg_.linear.x = k_u_*(starting_point_.x - current_pos_.x);

    // Mw_Xkp1_ = Mw_A_*Mw_Xk_ + Mw_B_*u_z_;
    // u_w_ = Mw_C_*Mw_Xkp1_;
    // Mw_Xk_ = Mw_Xkp1_;

    cmd_vel_msg_.linear.y = k_v_ * (goal_point_.y - current_pos_.y);
    // cmd_vel_msg_.linear.y = 1.0;

    cmd_vel_msg_.linear.z = k_w_ * (goal_point_.z - current_pos_.z);
    // cmd_vel_msg_.linear.z = 1.0;

    cmd_vel_msg_.angular.z = (k_r_ / 50.0) * (goal_heading_ - current_heading_);
    cmd_vel_msg_.angular.z = 1.0;

    // float e_r = (goal_heading_ - current_heading_)*c1_;
    // Mr_Xkp1_ = Mr_A_*Mr_Xk_ + Mr_B_*e_r;
    // u_r_ = Mr_C_*Mr_Xkp1_;
    // Mr_Xk_ = Mr_Xkp1_;
    // // ROS_INFO("u_r: %f, psi: %f", u_r_, u_psi);
    // cmd_vel_msg_.angular.z = u_r_;
  }
}

void commandGenerator::generateSwerveCommands() {

  if (!start_fwd_motion_) {
    fwd_motion_time_ = ros::Time::now();
    start_fwd_motion_ = true;
    start_swerve_ = false;
  }

  float durr = (ros::Time::now() - fwd_motion_time_).toSec();
  bool forward_motion_steady = false;
  if (durr > 4.0) {
    forward_motion_steady = true;
  }

  if (!start_swerve_ && forward_motion_steady) {
    cmd_vel_msg_.linear.y = 0.0;
    cmd_vel_msg_.angular.z = 0.0;
    start_swerve_ = true;
    swerve_complete_ = false;
    start_swerve_time_ = ros::Time::now();
  }

  float t_elapsed = 0.0;
  t_elapsed = (ros::Time::now() - start_swerve_time_).toSec();

  float cmd = 0.0;
  if (!swerve_complete_ && forward_motion_steady) {
    // ROS_INFO_THROTTLE(1.0, "swerving...");
    cmd = .25 * sin(2.0 * M_PI / swerve_dur_ * t_elapsed);
    // ROS_INFO("cmd: %f",cmd);
  }

  if (start_swerve_ && t_elapsed > swerve_dur_) {
    swerve_complete_ = true;
  }

  cmd_vel_msg_.linear.y = cmd;
  cmd_vel_msg_.angular.z = -cmd;

  if (swerve_complete_) {
    cmd_vel_msg_.linear.y = 0.0;
    cmd_vel_msg_.angular.z = 0.0;
  }
}

void commandGenerator::generateDoubletsCommand() {
  if (!start_doublets_) {
    start_doublets_ = true;
    start_doublets_time_ = ros::Time::now();
    first_doublet_complete_ = false;
  }

  float time_diff_s = (ros::Time::now() - start_doublets_time_).toSec();
  if (time_diff_s > doublet_period_ && !first_doublet_complete_) {
    first_doublet_complete_ = true;
    start_doublets_time_ = ros::Time::now();
    time_diff_s = (ros::Time::now() - start_doublets_time_).toSec();
  }

  float cmd;
  if (!first_doublet_complete_) {
    // First doublet
    if (time_diff_s <= doublet_period_ / 4.0) {
      cmd = doublet_amplitude_;
    } else if ((time_diff_s >= doublet_period_ / 4.0) &&
               (time_diff_s <= doublet_period_ / 2.0)) {
      cmd = -doublet_amplitude_;
    } else if ((time_diff_s >= doublet_period_ / 2.0) &&
               (time_diff_s <= 3.0 * doublet_period_ / 4.0)) {
      cmd = doublet_amplitude_;
    } else if ((time_diff_s >= 3.0 * doublet_period_ / 4.0) &&
               (time_diff_s <= doublet_period_)) {
      cmd = -doublet_amplitude_;
    }

  } else {
    // Second doublet
    if (time_diff_s <= doublet_period_ / 8.0) {
      cmd = doublet_amplitude_;
    } else if ((time_diff_s >= doublet_period_ / 8.0) &&
               (time_diff_s <= doublet_period_ / 4.0)) {
      cmd = -doublet_amplitude_;
    } else if ((time_diff_s >= doublet_period_ / 4.0) &&
               (time_diff_s <= 3.0 * doublet_period_ / 8.0)) {
      cmd = doublet_amplitude_;
    } else if ((time_diff_s >= 3.0 * doublet_period_ / 8.0) &&
               (time_diff_s <= doublet_period_ / 2.0)) {
      cmd = -doublet_amplitude_;
    } else if (time_diff_s >= doublet_period_ / 2.0) {
      state_ = "init";
    }
  }

  // cmd_vel_msg_.linear.y = cmd;
  cmd_vel_msg_.linear.z = cmd;
  // cmd_vel_msg_.angular.z = cmd;
}

} // namespace command_generator
