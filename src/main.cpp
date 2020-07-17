#include "ros-class-node/node_class.hpp"


const std::string RosnodeName = "node_class_server";

int main(int argc, char ** argv)
{
    // Initialize rosnode
    ros::init(argc, argv, RosnodeName);

    // Create node_class obj
    NODE_CLASS::node_class my_node("signal_state", "data_sender");

    my_node.start();

    return 0;
}