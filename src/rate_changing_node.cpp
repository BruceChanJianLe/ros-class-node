#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "dynamic_reconfigure/server.h"
#include "ros-class-node/RateChangingNodeConfig.h"

class RateChangingNode
{
public:
  RateChangingNode(ros::NodeHandle &nh)
  : count_(0)
  , rate_(ros::Rate(1))
  {
    pub_ = nh.advertise<std_msgs::Int64>("cmd_vel", 1);
    server_.setCallback([this](ros_class_node::RateChangingNodeConfig& config, ...)
    {
      rate_ = ros::Rate(config.rate_update);
      ROS_INFO_STREAM("Rate updated to " << config.rate_update);
    });
  }

  ~RateChangingNode() {}

  void pub()
  {
    std_msgs::Int64 msg;
    msg.data = count_;
    ROS_INFO_STREAM("Publishing count: " << count_);
    pub_.publish(msg);
    ++count_;
  }

  int64_t count_;
  ros::Publisher pub_;
  ros::Rate rate_;
  dynamic_reconfigure::Server<ros_class_node::RateChangingNodeConfig> server_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rate_changing_node");
  ros::NodeHandle nh;

  RateChangingNode node(nh);

  while (nh.ok())
  {
    node.pub();
    ros::spinOnce();
    node.rate_.sleep();
  }

  return 0;
}
