#include <nearness_control/joy_teleop_mixer.h>

namespace joy_teleop_mixer{
joyTeleopMixer::joyTeleopMixer(const ros::NodeHandle &node_handle,
                                       const ros::NodeHandle &private_node_handle)
      :nh_(node_handle),
       pnh_(private_node_handle){
      this->init();
  }

void joyTeleopMixer::init() {

    //priv_nh.param("desired_altitude", ref_h_);
    //priv_nh.param("k_height", k_height_);
    //ROS_INFO("alt: %f, k: %f",  ref_h_, k_height_);

    sub_joy_ = nh_.subscribe("joy", 1, &joyTeleopMixer::joyconCb, this);

    pub_cmd_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    pnh_.param("max_forward_speed", max_forward_speed_, 1.0);
    pnh_.param("max_lateral_speed", max_lateral_speed_, 1.0);
    pnh_.param("max_vertical_speed", max_vertical_speed_, .5);
    pnh_.param("max_yaw_rate", max_yaw_rate_, .5);


    max_forward_speed_ = 1.0;
    max_lateral_speed_ = 1.0;
    max_vertical_speed_ = 1.0;
    max_yaw_rate_ = 1.0;

    flag_alt_hold_ = false;

}

void joyTeleopMixer::joyconCb(const sensor_msgs::JoyConstPtr& joy_msg)
{
    //ROS_INFO_THROTTLE(1,"joy cb");
    last_joy_msg_time_ = ros::Time::now();
    joy_ = *joy_msg;

    if(joy_.buttons[5] == 1){
        flag_alt_hold_ = true;
        ROS_INFO_THROTTLE(2,"ALT HOLD");
    }

    geometry_msgs::Twist cmd_out;
    if(joy_.buttons[4] == 1){
        cmd_out.linear.x = joy_.axes[4]*max_forward_speed_;
        cmd_out.linear.y = joy_.axes[3]*max_lateral_speed_;
        cmd_out.linear.z = joy_.axes[1]*max_vertical_speed_;
        if(flag_alt_hold_){
            cmd_out.linear.z = 0;
        }
        cmd_out.angular.z = joy_.axes[0]*max_yaw_rate_;
        pub_cmd_.publish(cmd_out);
    } else {
        pub_cmd_.publish(cmd_out);
    }

}


 // end of class
} // End of namespace nearness
