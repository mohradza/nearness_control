#include <nearness_control/control_mixer.h>

namespace control_mixer{
controlMixer::controlMixer(const ros::NodeHandle &node_handle,
                                       const ros::NodeHandle &private_node_handle)
      :nh_(node_handle),
       pnh_(private_node_handle){
      this->init();
  }

void controlMixer::init() {

    //priv_nh.param("desired_altitude", ref_h_);
    //priv_nh.param("k_height", k_height_);
    //ROS_INFO("alt: %f, k: %f",  ref_h_, k_height_);

    sub_joy_ = nh_.subscribe("joy_vel", 1, &controlMixer::joyconCb, this);

    sub_odom_ = nh_.subscribe("odometry", 1, &controlMixer::odomCb, this);

    pub_cmd_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    ref_h_ = 1.0;
    ref_heading_ = 0.0;
    k_height_ = .25;
    k_heading_ = .25;
    enable_joy_thrust_control_ = false;
    enable_heading_controller_ = false;
}

void controlMixer::joyconCb(const geometry_msgs::TwistConstPtr& joy_msg)
{
     last_joy_msg_time_ = ros::Time::now();
     joy_cmd_ = *joy_msg;

}

void controlMixer::odomCb(const nav_msgs::OdometryConstPtr& odom_msg)
{
    odom_ = *odom_msg;
    float alt_err = ref_h_ - odom_.pose.pose.position.z;

    geometry_msgs::Quaternion quat_msg = odom_.pose.pose.orientation;
    tf::Quaternion quat_tf;
    tf::quaternionMsgToTF(quat_msg, quat_tf);
    double roll;
    double pitch;
    double current_heading;
    tf::Matrix3x3(quat_tf).getRPY(roll, pitch, current_heading);

    float heading_err = ref_heading_ - current_heading;
    //ROS_INFO("%f, %f", ref_h_, odom_.pose.pose.position.z);

    // if((ros::Time::now() - last_joy_msg_time_).toSec() > .05){
    //     joy_cmd_.linear.x = 0.0;
    //     joy_cmd_.linear.y = 0.0;
    //     joy_cmd_.linear.z = 0.0;
    //     joy_cmd_.angular.z = 0.0;
    // }

    geometry_msgs::Twist cmd_out;
    cmd_out.linear.x = joy_cmd_.linear.x;
    cmd_out.linear.y = joy_cmd_.linear.y;
    if((abs(joy_cmd_.linear.z) > 0.0) && enable_joy_thrust_control_){
      cmd_out.linear.z = joy_cmd_.linear.z;
    } else {
      cmd_out.linear.z = k_height_*alt_err;
    }
    if(enable_heading_controller_){
      cmd_out.angular.z = k_heading_*heading_err;
    } else {
      cmd_out.angular.z = joy_cmd_.angular.z;
    }

    pub_cmd_.publish(cmd_out);

}


 // end of class
} // End of namespace nearness
