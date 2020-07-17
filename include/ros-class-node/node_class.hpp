#ifndef __NODE_CLASS_H_
#define __NODE_CLASS_H_

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

#include <string>

#ifdef DEBUG_
    #include <ncurses.h>
#endif


namespace NODE_CLASS
{

    // Create node class
    class node_class
    {
        private:
            // ROS declarations
            ros::NodeHandle private_nh_;
            ros::NodeHandle relative_nh_;
            ros::NodeHandle n_;
            ros::Publisher pub_;
            ros::Subscriber sub_;
            
            // ROS msg declaration
            std_msgs::Bool cur_msg_;

            // Variable declarations
            int threshold_;
            bool data_receive_;
            int rate_;
            unsigned int data_;

            // Private function
            void init();
            void process_data();

            #ifdef DEBUG_
                // ncurses window
                WINDOW * win;
            #endif

        public:
            // Constructor
            node_class(std::string pub_name, std::string sub_name);

            node_class();

            // Destructor
            ~node_class();

            // subscriber callback function
            void sub_callback(const std_msgs::Int8::ConstPtr &);

            // Start node_class function
            void start();
    };

} // namespace NODE_CLASS

#endif