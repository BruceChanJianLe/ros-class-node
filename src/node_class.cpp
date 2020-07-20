#include "ros-class-node/node_class.hpp"


namespace NODE_CLASS
{
    // node_class constructor
    node_class::node_class(std::string pub_name, std::string sub_name)
    : private_nh_("~"),
    pub_(relative_nh_.advertise<std_msgs::Bool>(pub_name, 100)),
    sub_(relative_nh_.subscribe<std_msgs::Int8>(sub_name, 100, & node_class::sub_callback, this))
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
            int max_row, max_col;
            getmaxyx(stdscr, max_row, max_col);

            // Initialize window
            win = newwin(13, max_col, 7, 0);
            box(win, 0, 0);
        #endif

        // Initialize parameters (For ROS params)
        init();

        // Inform user
        ROS_INFO_STREAM("Node_class initialized!");

        #ifdef DEBUG_
            // Display buffer messages
            refresh();
        #endif
    }


    // node_class constructor
    node_class::node_class()
    {
        // Initialize parameters (For ROS params)
        init();
    }


    // node_class destructor
    node_class::~node_class()
    {
        #ifdef DEBUG_
            delwin(win);
            endwin();
        #endif
    }


    // Initialization function
    // Because we cannot initialize ROS params with initializer list
    void node_class::init()
    {
        // Initialize ROS param
        private_nh_.param("m_threshold", threshold_, 50);
        private_nh_.param("ros_node_publish_rate", rate_, 5);

        // Initialize ROS msg
        cur_msg_.data = false;

        #ifdef DEBUG_
            mvprintw(2, 0, "threshold:%d", threshold_);
            mvprintw(3, 0, "publish_rate:%d", rate_);
        #endif

    }


    // Subscriber callback function
    void node_class::sub_callback(const std_msgs::Int8::ConstPtr & msg)
    {
        // Pass msg value to local variable
        data_ = msg->data;

        #ifdef DEBUG_
            // // Display in terminal
            // ROS_INFO_STREAM("Message received!");
            wattron(win, A_BOLD | A_UNDERLINE);
            mvwprintw(win, 1, 2, "received data");
            wattroff(win, A_BOLD | A_UNDERLINE);
            mvwprintw(win, 2, 2, "data: %d", data_);
        #endif

        // Set data_receive_ as true
        data_receive_ = true;
    }


    // Start node_class function
    void node_class::start()
    {
        // Set ROS sleep rate
        ros::Rate r(rate_);

        while(relative_nh_.ok())
        {
            // Process data
            this->process_data();

            // ROS spin
            ros::spinOnce();

            // Sleep
            r.sleep();
        }
    }


    // Process received data
    void node_class::process_data()
    {
        #ifdef DEBUG_
            wattron(win, A_BOLD | A_UNDERLINE);
            mvwprintw(win, 1, 20, "result");
            wattroff(win, A_BOLD | A_UNDERLINE);
        #endif
    
        if(data_receive_)
        {
            cur_msg_.data = threshold_function(data_);
            #ifdef DEBUG_
                mvwprintw(win, 2, 20, "result: %s", (cur_msg_.data) ? "true " : "false");
            #endif
        }

        #ifdef DEBUG_
            wrefresh(win);
        #endif

        // Publish data to topic
        pub_.publish(cur_msg_);
    }

    // For rostest purpose
    bool node_class::threshold_function(const int data)
    {
        if(data > threshold_)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

} // namespace NODE_CLASS