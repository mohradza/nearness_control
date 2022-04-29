#include "nearness_control/data_collector.h"
#include <string>

int main(int argc, char** argv) {
    std::string node_name = "data_collector";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("data_collector");
    ros::NodeHandle nh_private("~");
    data_collector::dataCollector data_collector_node(nh, nh_private);

    while(ros::ok()){
      ros::spin();
    }
}
