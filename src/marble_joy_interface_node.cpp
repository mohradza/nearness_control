#include "nearness_control/marble_joy_interface.h"
#include <string>

int main(int argc, char** argv) {
    std::string node_name = "marble_joy_interface";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("marble_joy_interface");
    ros::NodeHandle nh_private("~");
    //nearness::NearnessController nearness_controller_node(nh, nh_private);
    marble_joy_interface::marbleJoyInterface marble_joy_interface_node(nh, nh_private);
    ros::spin();
}
