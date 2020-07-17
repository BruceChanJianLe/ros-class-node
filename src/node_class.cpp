#include "ros-class-node/node_class.hpp"


// node_class constructor
node_class::node_class(std::string pub_name, std::string sub_name, int threshold_value)
: private_nh_("~"),
  pub_(relative_nh_.advertise<std_msgs::Bool>(pub_name, 100)),
  sub_(relative_nh_.subscribe<std_msgs::Int8>(sub_name, 100, & node_class::sub_callback, this)),
  threshold(threshold_value)
{
    #ifdef DEBUG_
        // Initializes the ncurses screen
        initscr();
        // Ctrl+c to quit ncurses
        cbreak();
        // Do not print user input values
        noecho();

        // ROS node name
        attron(A_STANDOUT | A_BOLD);
        mvprintw(0, 0, "node_class_server");
        attroff(A_STANDOUT | A_BOLD);

        // Obtain max row and column for window
        int max_row, max_col
        getmaxyx(stdscr, max_row, max_col);

        // Initialize window
        win = newwin(13, max_col, 10, 0);
        box(win, 0, 0)

    #endif
    ROS_INFO_STREAM("Node_class initialized!");
}


// Subscriber callback function
void node_class::sub_callback(const std_msgs::Int8::ConstPtr & msg)
{
    // Display in terminal
    ROS_INFO_STREAM("Message received!");

    // Create a std_msgs message to hold our message
    std_msgs::Bool current_msg;

    // Processing our message
    if(msg->data > this->threshold)
        current_msg.data = true;
    else
        current_msg.data = false;
    
    // Publish to topic
    pub_.publish(current_msg);
}