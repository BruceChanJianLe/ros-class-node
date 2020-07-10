# ROS Class Node Version 1.0

This repo shows an example of rosnode using class. You are able to publish a topic inside a callback function of a subscriber.  
We will be creating a `data_sender` node and a `node_class` node to recieve the data send by `data_sender` node.

### Explanation

data_sender_node -(/data_sender)-> node_class_server -(/signal_state)->

data_sender_node will publish int from 0 to inf, once the threshold reaches 51 /signal_state will change to true from false.

**Version 1.0**  
- add in DEBUG_ mode which uses ncurses to display more information of the node.  
- add private and relative ros node handle (private to load rosparam and relative to subscribe other rosnode)
- add yaml file to load rosparam
- test on Ubuntu 18 with ROS melodic


## Step 1

**Create a catkin package**

```bash
catkin_create_pkg ros-class-node roscpp rospy std_msgs message_generation message_runtime
```

## Step 2


**Writing the code according to the example given**
```bash
.
├── CMakeLists.txt
├── include
│   └── ros-class-node
│       └── node_class.hpp    # node_class header
├── LICENSE
├── package.xml
├── README.md
└── src
    ├── data_sender.cpp     # data_sender_node source file
    ├── main.cpp            # node_class main function use
    └── node_class.cpp      # node_class source file

3 directories, 8 files
```

## Step 3

**Changes done to CMakeLists.txt**

```cmake
## Include the include directory so that you can find the header file
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Add node_class executable
add_executable(node_class
               src/main.cpp
               src/node_class.cpp
              )

## Link node_class with catkin libraries
target_link_libraries(node_class ${catkin_LIBRARIES})

## Add data_sender executable
add_executable(data_sender_node
               src/data_sender.cpp
              )

## Link data_sender_node with catkin libraries
target_link_libraries(data_sender_node ${catkin_LIBRARIES})
```

# All in one
```cpp
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <string>

const std::string RosnodeName = "node_class_server";

class node_class
{
    private:
        ros::NodeHandle n_;
        ros::Publisher pub_;
        ros::Subscriber sub_;
        int threshold;
    public:
        node_class(std::string pub_name, std::string sub_name, int threshold_value);
        void sub_callback(const std_msgs::Int8::ConstPtr &);
};

// node_class constructor
node_class::node_class(std::string pub_name, std::string sub_name, int threshold_value)
: pub_(n_.advertise<std_msgs::Bool>(pub_name, 100)),
  sub_(n_.subscribe<std_msgs::Int8>(sub_name, 100, & node_class::sub_callback, this)),
  threshold(threshold_value)
{
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
```

# Reference 
- Publishing to a topic via subscriber callback function [link](https://answers.ros.org/question/59725/publishing-to-a-topic-via-subscriber-callback-function/)