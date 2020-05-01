#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <memory>


int main(int argc, char ** argv)
{
    // Initialize rosnode
    ros::init(argc, argv, "data_sender_node");

    // Create a rosnode handler
    ros::NodeHandle n;

    // Create a publisher
    auto pub = n.advertise<std_msgs::Int8>("data_sender", 100);

    // Create a sleep method to control the rate (5 times per second)
    ros::Rate r(1);

    // Create a share ptr for the message
    auto msg_ptr = std::make_shared<std_msgs::Int8>();
    msg_ptr->data = 0;
    int count = 0;

    while(ros::ok())
    {
        // Check if data is out of scope
        if(msg_ptr->data == 255)
            msg_ptr->data = 0;

        // Put information inside message
        msg_ptr->data++;

        // Place the message in buffer
        pub.publish(* msg_ptr);

        // Let ros process the message once
        ros::spinOnce();

        // Display in terminal
        ROS_INFO_STREAM("#" << count << " data sent!");
        count++;

        // Sleep the thread (main thread)
        r.sleep();
    }

    return 0;
}