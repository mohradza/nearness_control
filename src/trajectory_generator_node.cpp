#include "nearness_control/trajectory_generator.h"
#include <string>

int main(int argc, char** argv) {
    std::string node_name = "trajectory_generator";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("trajectory_generator");
    ros::NodeHandle nh_private("~");
    //nearness::NearnessController nearness_controller_node(nh, nh_private);
    trajectory_generator::trajectoryGenerator trajectory_generator_node(nh, nh_private);

    ros::Rate loop_rate(10);

    while(ros::ok()){
        trajectory_generator_node.publishTrajectory();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
