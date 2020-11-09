#include "nearness_control/nearness_controller_3d.h"
#include <string>

int main(int argc, char** argv) {
    std::string node_name = "nearness_controller_3d";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("nearness_controller_3d");
    ros::NodeHandle nh_private("~");
    nearness_3d::NearnessController3D nearness_control_3d_node(nh, nh_private);

    ros::Rate loop_rate(100);

    while(ros::ok()){

      nearness_control_3d_node.publishProjectionShapes();

        if(nearness_control_3d_node.newPcl()){
            // Process 3D nearness
            nearness_control_3d_node.projectNearness();
            nearness_control_3d_node.reconstructWideFieldNearness();
            //nearness_control_3d_node.computeSmallFieldNearness();
            nearness_control_3d_node.computeControlCommands();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

}
