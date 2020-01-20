#include <nearness_control/marble_joy_interface.h>

namespace marble_joy_interface{
marbleJoyInterface::marbleJoyInterface(const ros::NodeHandle &node_handle,
                                       const ros::NodeHandle &private_node_handle)
      :nh_(node_handle),
       pnh_(private_node_handle) {
      this->init();
  }

void marbleJoyInterface::init() {

    sub_bluetooth_joy_ = nh_.subscribe("joy", 1, &marbleJoyInterface::joyconCb, this);

    pub_estop_engage_ = nh_.advertise<std_msgs::Bool>("estop_cmd", 10);

    pub_radio_estop_disengage_ = nh_.advertise<std_msgs::Bool>("radio_reset_cmd", 1);
}

void marbleJoyInterface::joyconCb(const sensor_msgs::JoyConstPtr& joy_msg)
{

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

    if(joy_msg->buttons[3] == 1){
        pub_radio_estop_disengage_.publish("false");
    }

}


 // end of class
} // End of namespace nearness
