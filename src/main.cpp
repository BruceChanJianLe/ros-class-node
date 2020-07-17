#include "ros-class-node/node_class.hpp"


const std::string RosnodeName = "node_class_server";
const std::string pub_name = "signal_state";
const std::string sub_name = "data_sender";

int main(int argc, char ** argv)
{
    // Initialize rosnode
    ros::init(argc, argv, RosnodeName);

    // Create node_class obj
    NODE_CLASS::node_class my_node(pub_name, sub_name);

    my_node.start();

    return 0;
}