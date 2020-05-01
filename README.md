# ROS Class Node

This repo shows an example of rosnode using class. You are able to publish a topic inside a callback function of a subscriber.  
We will be creating a `data_sender` node and a `node_class` node to recieve the data send by `data_sender` node.


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
│       └── node_class.h    # node_class header
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

# Reference 
- Publishing to a topic via subscriber callback function [link](https://answers.ros.org/question/59725/publishing-to-a-topic-via-subscriber-callback-function/)