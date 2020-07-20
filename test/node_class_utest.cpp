// ROS header file
#include <ros/ros.h>

// Google test header file
#include <gtest/gtest.h>

// To be tested header file
#include "ros-class-node/node_class.hpp"


class node_class_utest : public ::testing::Test
{
    protected:
        NODE_CLASS::node_class test_node;
};


TEST_F(node_class_utest, threshold_test_true)
{
    // Test threshold function
    int data = 21;
    bool result = test_node.threshold_function(data);
    ASSERT_TRUE(result);
}


TEST_F(node_class_utest, threshold_test_false)
{
    // Test threshold function
    int data = 20;
    bool result = test_node.threshold_function(data);
    ASSERT_FALSE(result);
}


int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "node_class_test");

    return RUN_ALL_TESTS();
}