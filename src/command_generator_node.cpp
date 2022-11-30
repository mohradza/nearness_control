#include "nearness_control/command_generator.h"
#include <string>

int main(int argc, char **argv) {
  std::string node_name = "command_generator";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("command_generator");
  ros::NodeHandle nh_private("~");
  command_generator::commandGenerator command_generator_node(nh, nh_private);

  ros::Rate loop_rate(200);

  while (ros::ok()) {

    // command_generator_node.publishControlCommands();

    ros::spinOnce();
    loop_rate.sleep();
  }
}
