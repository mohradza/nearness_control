#include "nearness_control/nearness_controller.h"
#include <string>

int main(int argc, char** argv) {
    std::string node_name = "nearness_controller";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("nearness_controller");
    ros::NodeHandle nh_private("~");
    nearness::NearnessController nearness_controller_node(nh, nh_private);
    ros::spin();
}
