#include "nearness_control/control_mixer.h"
#include <string>

int main(int argc, char** argv) {
    std::string node_name = "control_mixer";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("control_mixer");
    ros::NodeHandle nh_private("~");
    //nearness::NearnessController nearness_controller_node(nh, nh_private);
    control_mixer::controlMixer control_mixer_node(nh, nh_private);
    ros::spin();
}
