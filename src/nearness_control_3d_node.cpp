#include "nearness_control/nearness_control_3d.h"
#include <string>

int main(int argc, char **argv) {
  std::string node_name = "nearness_control_3d";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("nearness_control_3d");
  ros::NodeHandle nh_private("~");
  NearnessControl3D nearness_control_3d_node(nh, nh_private);

  while (ros::ok()) {
    ros::spin();
  }
}
