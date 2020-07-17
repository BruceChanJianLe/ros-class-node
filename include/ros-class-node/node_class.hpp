#ifndef __NODE_CLASS_H_
#define __NODE_CLASS_H_

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <string>

// #ifdef DEBUG_
    #include <ncurses.h>
// #endif


// Create node class
class node_class
{
    private:
        ros::NodeHandle private_nh_;
        ros::NodeHandle relative_nh_;
        ros::NodeHandle n_;
        ros::Publisher pub_;
        ros::Subscriber sub_;
        int threshold;
        bool data_receive;
        void process_data();
    public:
        node_class(std::string pub_name, std::string sub_name, int threshold_value);
        void sub_callback(const std_msgs::Int8::ConstPtr &);
};

#endif