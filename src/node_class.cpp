#include "ros-class-node/node_class.hpp"


namespace NODE_CLASS
{
    // node_class constructor
    node_class::node_class(std::string pub_name, std::string sub_name)
    : private_nh_("~"),
    pub_(relative_nh_.advertise<std_msgs::Bool>(pub_name, 100)),
    sub_(relative_nh_.subscribe<std_msgs::Int8>(sub_name, 100, & node_class::sub_callback, this))
    {
        // Initialize parameters (For ROS params)
        init();

        // Inform user
        ROS_INFO_STREAM("Node_class initialized!");
    }


    // node_class constructor
    node_class::node_class()
    {
        return;
    }


    // node_class destructor
    node_class::~node_class()
    {
        ;
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

    }


    // Subscriber callback function
    void node_class::sub_callback(const std_msgs::Int8::ConstPtr & msg)
    {
        // Pass msg value to local variable
        data_ = msg->data;

        #ifdef DEBUG_
            // Display in terminal
            ROS_INFO_STREAM("Message received!");
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
        if(data_receive_)
        {
            if(data_ > threshold_)
                cur_msg_.data = true;
            else
                cur_msg_.data = false;
        }

        // Publish data to topic
        pub_.publish(cur_msg_);
    }
} // namespace NODE_CLASS