#include "ros-class-node/node_class.h"


const std::string RosnodeName = "node_class_server";

int main(int argc, char ** argv)
{
    // Initialize rosnode
    ros::init(argc, argv, RosnodeName);

    // Create node_class obj
    node_class my_node("signal_state", "data_sender", 50);

    // Let ros do its thing
    ros::spin();

    return 0;
}