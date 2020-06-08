#include "nearness_control/joy_teleop_mixer.h"
#include <string>

int main(int argc, char** argv) {
    std::string node_name = "joy_teleop_mixer";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("joy_teleop_mixer");
    ros::NodeHandle nh_private("~");
    //nearness::NearnessController nearness_controller_node(nh, nh_private);
    joy_teleop_mixer::joyTeleopMixer joy_teleop_mixer_node(nh, nh_private);
    ros::spin();
}
