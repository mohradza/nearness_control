#include "nearness_control/trajectory_follower.h"
#include <string>

int main(int argc, char** argv) {
    std::string node_name = "trajectory_follower";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("trajectory_follower");
    ros::NodeHandle nh_private("~");
    //nearness::NearnessController nearness_controller_node(nh, nh_private);
    trajectory_follower::trajectoryFollower trajectory_follower_node(nh, nh_private);

    ros::Rate loop_rate(10);

    while(ros::ok()){
        if(trajectory_follower_node.doLookup()){
            trajectory_follower_node.publishLookahead();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
